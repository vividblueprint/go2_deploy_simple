"""One Go2 state machine and observation builder for the existing TS/EE/WaQ models."""
from collections import deque
from pathlib import Path
import struct

import numpy as np
import torch

from unitree_sdk2_go2 import low_command

ROOT = Path(__file__).resolve().parent
POLICIES = {
    'ts': ('policy_ts_gs.pt', 20, list(range(12))),
    'ee': ('Dec15_14-57-47_gym_ee_policy.pt', 10, [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]),
    'waq': ('Dec16_09-19-17_gym_dreamwaq_policy.pt', 20, [3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]),
}
CONTROL_DT = 0.02
COMMUNICATION_DT = 0.002
TRANSITION_TIME = 2.0
# DDS motor order: FR, FL, RR, RL. Bounds follow the bundled Go2 model.
DDS_JOINT_MIN = np.array([-1.0472, -1.5708, -2.7227] * 2 + [-1.0472, -0.5236, -2.7227] * 2)
DDS_JOINT_MAX = np.array([1.0472, 3.4907, -0.83776] * 2 + [1.0472, 4.5379, -0.83776] * 2)
DEFAULT_ANGLES = np.array([0.0, 0.8, -1.5] * 4, dtype=np.float32)
# Stay inside the MuJoCo calf joint limit during the initialization pose.
SIT_ANGLES = np.array([0.04, 1.3, -2.7, -0.02, 1.29, -2.7,
                       -0.33, 1.27, -2.7, 0.32, 1.28, -2.7], dtype=np.float32)


def remote_input(data):
    data = bytes(data)
    if len(data) < 24:
        raise ValueError('Wireless input must contain at least 24 bytes')
    keys = struct.unpack_from('<H', data, 2)[0]
    lx, rx = struct.unpack_from('<ff', data, 4)
    ly = struct.unpack_from('<f', data, 20)[0]
    command = np.array([ly, -lx, -rx], dtype=np.float32)
    if not np.isfinite(command).all():
        raise ValueError('Wireless input contains nonfinite joystick values')
    return keys, np.clip(command, -1.0, 1.0)


def gravity(quat):
    w, x, y, z = quat
    return np.array([2 * (-z*x + w*y), -2 * (z*y + w*x),
                     1 - 2 * (w*w + z*z)], dtype=np.float32)


class PolicyController:
    def __init__(self, name='ts', real=False):
        filename, frames, indices = POLICIES[name]
        self.name = name
        self.real = real
        self.armed = False
        self.stand_gains = (60.0, 5.0) if real else (40.0, 1.0)
        self.damping_kd = 4.0 if real else 1.0
        self.indices = indices
        torch.set_num_threads(1)
        self.policy = torch.jit.load(str(ROOT / 'models' / filename), map_location='cpu').eval()
        self.history = deque([np.zeros(45, dtype=np.float32) for _ in range(frames)], maxlen=frames)
        self.action = np.zeros(12, dtype=np.float32)
        self.command = low_command()
        self.state = 'zero'
        self.previous_keys = 0
        self.transition_start = 0.0
        self.transition_initial = np.zeros(12, dtype=np.float32)

    def damping(self):
        if self.state != 'damping':
            print('State: damping', flush=True)
        self.state = 'damping'
        for motor in self.command.motor_cmd:
            motor.q, motor.dq, motor.kp, motor.kd, motor.tau = 0.0, 0.0, 0.0, self.damping_kd, 0.0
        return self.command

    def disarm(self):
        self.armed = False
        return self.damping()

    def _transition(self, state, low_state, now):
        self.state = state
        self.transition_start = now
        self.transition_initial = np.array([low_state.motor_state[i].q for i in self.indices])
        print('State:', state, flush=True)

    def _buttons(self, keys, low_state, now):
        rising = keys & ~self.previous_keys
        self.previous_keys = keys
        if not keys & (1 << 1):  # L1 modifier from the real remote or simulated keyboard.
            return
        if rising & (1 << 11):  # 1: damping from any state.
            self.armed = True
            self.damping()
        elif rising & (1 << 10) and self.state == 'damping':
            self.armed = False
            self.state = 'zero'
            self.command = low_command()
            print('State: zero torque', flush=True)
        elif self.real and not self.armed:
            return
        elif rising & (1 << 0) and self.state == 'damping':
            self._transition('sit', low_state, now)
        elif rising & (1 << 4) and self.state == 'sit' and now - self.transition_start >= TRANSITION_TIME:
            self._transition('stand', low_state, now)
        elif rising & (1 << 8) and self.state == 'stand' and now - self.transition_start >= TRANSITION_TIME:
            self.state = 'walk'
            self.action.fill(0)
            for frame in self.history:
                frame.fill(0)
            print('State: walk', flush=True)

    def observation(self, low_state, velocity_command):
        q = np.array([low_state.motor_state[i].q for i in self.indices], dtype=np.float32)
        dq = np.array([low_state.motor_state[i].dq for i in self.indices], dtype=np.float32)
        obs = np.concatenate([
            velocity_command * np.array([1.0, 1.0, 0.25], dtype=np.float32),
            gravity(low_state.imu_state.quaternion),
            np.array(low_state.imu_state.gyroscope, dtype=np.float32) * 0.25,
            q - DEFAULT_ANGLES, dq * 0.05, self.action,
        ]).astype(np.float32)
        return obs

    def _infer(self, low_state, velocity_command):
        obs = self.observation(low_state, velocity_command)
        self.history.append(obs)
        history = torch.from_numpy(np.concatenate(self.history)).unsqueeze(0)
        with torch.no_grad():
            output = self.policy(history) if self.name == 'ee' else self.policy(torch.from_numpy(obs).unsqueeze(0), history)
        action = output.reshape(-1).detach().cpu().numpy()
        if action.shape != (12,) or not np.isfinite(action).all():
            raise ValueError('Policy produced invalid motor actions')
        self.action = action.copy()
        return DEFAULT_ANGLES + 0.25 * self.action

    def update(self, low_state, now):
        keys, velocity_command = remote_input(low_state.wireless_remote)
        self._buttons(keys, low_state, now)
        if self.real:
            velocity_command *= 0.25
        if self.state == 'zero':
            return self.command
        if self.state == 'damping':
            return self.damping()
        if self.state == 'walk':
            target = self._infer(low_state, velocity_command)
            kp, kd = 20.0, 0.5
        else:
            alpha = np.clip((now - self.transition_start) / TRANSITION_TIME, 0.0, 1.0)
            pose = SIT_ANGLES if self.state == 'sit' else DEFAULT_ANGLES
            target = (1 - alpha) * self.transition_initial + alpha * pose
            kp, kd = self.stand_gains
        if self.real:
            target = np.clip(target, DDS_JOINT_MIN[self.indices], DDS_JOINT_MAX[self.indices])
        for i, motor_index in enumerate(self.indices):
            motor = self.command.motor_cmd[motor_index]
            motor.q, motor.dq, motor.tau = float(target[i]), 0.0, 0.0
            motor.kp, motor.kd = kp, kd
        return self.command
