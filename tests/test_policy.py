"""Verify preserved model behavior against the original calculate() implementations."""
import ast
from collections import deque
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np
import torch
from unitree_sdk2_go2 import LowState_, low_command, new_message

from policy import PolicyController, POLICIES, DEFAULT_ANGLES, remote_input

import os
ORIGINAL = Path(os.environ.get('GO2_ORIGINAL_DEPLOY', str(Path(__file__).resolve().parents[2] / 'go2_deploy_python')))


def original_calculate(name):
    source = ast.parse((ORIGINAL / 'controller.py').read_text())
    class_name = 'EEController' if name == 'ee' else 'TSController'
    cls = next(n for n in source.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'calculate')
    rotation = ast.parse((ORIGINAL / 'common/rotation_helper.py').read_text())
    gravity = next(n for n in rotation.body if isinstance(n, ast.FunctionDef) and n.name == 'get_gravity_orientation')
    namespace = {'np': np, 'torch': torch}
    exec(compile(ast.Module(body=[gravity, method], type_ignores=[]), 'original_calculate', 'exec'), namespace)
    return namespace['calculate']


class PolicyTests(unittest.TestCase):
    def test_all_models_match_original_actions_and_motor_order(self):
        rng = np.random.default_rng(8)
        for name in POLICIES:
            with self.subTest(policy=name):
                controller = PolicyController(name)
                state = new_message(LowState_)
                state.imu_state.quaternion = [1.0, 0.0, 0.0, 0.0]
                state.imu_state.gyroscope = [0.1, -0.2, 0.3]
                for motor in state.motor_state:
                    motor.q, motor.dq = rng.uniform(-1, 1), rng.uniform(-2, 2)
                reference = SimpleNamespace(
                    config=SimpleNamespace(leg_joint2motor_idx=controller.indices,
                        default_angles=DEFAULT_ANGLES, dof_pos_scale=1.0, dof_vel_scale=0.05,
                        ang_vel_scale=0.25, num_actions=12, cmd_scale=np.array([1., 1., .25]),
                        max_cmd=np.ones(3), action_scale=.25, ctrl_kp=20., ctrl_kd=.5),
                    qj=np.zeros(12, dtype=np.float32), dqj=np.zeros(12, dtype=np.float32),
                    action=np.zeros(12, dtype=np.float32), cur_obs=np.zeros(45, dtype=np.float32),
                    cmd=np.zeros(3), remote_controller=SimpleNamespace(ly=.4, lx=-.2, rx=.1),
                    low_state=state, low_cmd=low_command(), policy=controller.policy,
                    obs_deque=deque([np.zeros(45, dtype=np.float32) for _ in controller.history],
                                    maxlen=len(controller.history)))
                calculate = original_calculate(name)
                for _ in range(3):
                    calculate(reference)
                    target = controller._infer(state, np.array([.4, .2, -.1], dtype=np.float32))
                    np.testing.assert_allclose(controller.action, reference.action, atol=1e-6)
                    for i, motor_index in enumerate(controller.indices):
                        self.assertAlmostEqual(float(target[i]), reference.low_cmd.motor_cmd[motor_index].q, places=6)

    def test_keyboard_sequence_and_invalid_requests(self):
        import struct
        controller = PolicyController('ts')
        state = new_message(LowState_)
        state.imu_state.quaternion = [1., 0., 0., 0.]
        def press(bit, now):
            state.wireless_remote = list(bytearray(40))
            controller.update(state, now - .01)
            payload = bytearray(40)
            struct.pack_into('<H', payload, 2, (1 << 1) | (1 << bit))
            state.wireless_remote = list(payload)
            controller.update(state, now)
        press(8, 0)
        self.assertEqual(controller.state, 'zero')
        press(11, .1)
        self.assertEqual(controller.state, 'damping')
        press(0, .3)
        self.assertEqual(controller.state, 'sit')
        press(4, .5)
        self.assertEqual(controller.state, 'sit')
        press(4, 2.4)
        self.assertEqual(controller.state, 'stand')
        press(8, 4.5)
        self.assertEqual(controller.state, 'walk')
        press(11, 4.8)
        self.assertEqual(controller.state, 'damping')
        self.assertEqual(controller.command.motor_cmd[0].kp, 0)


if __name__ == '__main__':
    unittest.main()
