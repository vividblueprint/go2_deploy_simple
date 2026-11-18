"""Optional headless physics smoke check; requires mujoco in the test environment."""
from pathlib import Path
import struct
import sys

import mujoco
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from unitree_sdk2_go2 import LowState_, new_message
from policy import PolicyController, POLICIES


def run(name, scene):
    model = mujoco.MjModel.from_xml_path(str(scene))
    model.opt.timestep = 0.005
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    controller = PolicyController(name)
    state = new_message(LowState_)
    origin = None
    standing_height = None
    for n in range(2000):
        now = n * 0.005
        for i in range(12):
            state.motor_state[i].q = float(data.sensordata[i])
            state.motor_state[i].dq = float(data.sensordata[12+i])
        state.imu_state.quaternion = list(data.sensordata[36:40])
        state.imu_state.gyroscope = list(data.sensordata[40:43])
        remote = bytearray(40)
        for when, bit in [(1.0, 11), (1.3, 0), (3.5, 4), (5.7, 8)]:
            if when <= now < when + .15:
                struct.pack_into('<H', remote, 2, (1 << 1) | (1 << bit))
        if 7.0 <= now < 9.0:
            struct.pack_into('<f', remote, 20, .4)
        state.wireless_remote = list(remote)
        if n % 4 == 0:
            controller.update(state, now)
        for i in range(12):
            m = controller.command.motor_cmd[i]
            data.ctrl[i] = m.tau + m.kp * (m.q - data.sensordata[i]) + m.kd * (m.dq - data.sensordata[12+i])
        mujoco.mj_step(model, data)
        if n == 1100:
            standing_height = float(data.qpos[2])
        if n == 1400:
            origin = data.qpos[:3].copy()
    assert controller.state == 'walk', controller.state
    assert standing_height > .15, standing_height
    assert np.isfinite(data.qpos).all()
    assert data.qpos[2] > .15, data.qpos[:3]
    displacement = data.qpos[:3] - origin
    assert displacement[0] > .05, displacement
    print(f'{name}: stand height={standing_height:.3f}, final height={data.qpos[2]:.3f}, forward displacement={displacement[0]:.3f} m', flush=True)


if __name__ == '__main__':
    scene = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parents[2] / 'unitree_mujoco/unitree_robots/go2/scene.xml'
    for name in POLICIES:
        run(name, scene)
