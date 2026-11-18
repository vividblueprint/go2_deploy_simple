"""Run a Go2 TorchScript policy in MuJoCo or on a real Go2 over Ethernet."""
import argparse
import socket
import time

import numpy as np
from cyclonedds.internal import InvalidSample
from cyclonedds.util import duration
from unitree_sdk2_go2 import (
    Go2DDS, LowCmd_, LowState_, command_crc, MotionSwitcherClient, SportClient,
)

from policy import PolicyController, POLICIES, CONTROL_DT, COMMUNICATION_DT, remote_input


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--policy', choices=POLICIES, default='ts')
    parser.add_argument('--real', action='store_true', help='Control a real Go2 on DDS domain 0')
    parser.add_argument('--interface', help='Robot Ethernet interface, required with --real')
    args = parser.parse_args(argv)
    if args.real:
        if not args.interface or args.interface == 'lo':
            parser.error('--real requires --interface with the robot Ethernet interface (not lo)')
        try:
            socket.if_nametoindex(args.interface)
        except OSError:
            parser.error(f'Network interface not found: {args.interface}')
    elif args.interface not in (None, 'lo'):
        parser.error('An Ethernet interface requires --real; MuJoCo uses lo')
    args.interface = args.interface or 'lo'
    args.domain = 0 if args.real else 1
    return args


def validate_state(state):
    values = [v for motor in state.motor_state[:12] for v in (motor.q, motor.dq)]
    values += list(state.imu_state.quaternion) + list(state.imu_state.gyroscope)
    values += list(state.imu_state.accelerometer)
    if len(state.motor_state) < 12 or not np.isfinite(values).all():
        raise RuntimeError('Robot state contains missing or nonfinite joint/IMU data')
    if not 0.9 <= np.linalg.norm(state.imu_state.quaternion) <= 1.1:
        raise RuntimeError('Robot IMU quaternion is not initialized')
    remote_input(state.wireless_remote)
    return state


def wait_for_state(reader):
    deadline = time.monotonic() + 10
    while (remaining := deadline - time.monotonic()) > 0:
        try:
            state = reader.take_one(timeout=duration(seconds=remaining))
        except (TimeoutError, StopIteration):
            break
        if not isinstance(state, InvalidSample):
            return validate_state(state)
    raise RuntimeError('No valid robot state received within 10 seconds; check the interface and DDS domain')


def release_motion_control(dds):
    """Confirm mode release before creating the low-level command writer."""
    switcher = MotionSwitcherClient(dds, timeout=3.0)

    def active_mode():
        code, mode = switcher.CheckMode()
        if code != 0 or not isinstance(mode, dict) or not isinstance(mode.get('name'), str):
            raise RuntimeError(f'Cannot check robot motion mode (code {code}); no low-level commands sent')
        return mode['name']

    if not active_mode():
        return
    sport = SportClient(dds, timeout=3.0)
    code = sport.StandDown()
    if code != 0:
        raise RuntimeError(f'StandDown failed (code {code}); low-level control not started')
    time.sleep(1.0)
    for _ in range(3):
        code, _ = switcher.ReleaseMode()
        if code != 0:
            raise RuntimeError(f'ReleaseMode failed (code {code}); low-level control not started')
        time.sleep(0.5)
        if not active_mode():
            print('Built-in motion controller released.', flush=True)
            return
    raise RuntimeError('Built-in motion controller is still active; low-level control not started')


def run_controller(controller, reader, writer, state):
    last_state = time.monotonic()
    next_control = last_state
    stale = False
    try:
        while True:
            tick = time.monotonic()
            for sample in reader.take(32):
                if not isinstance(sample, InvalidSample):
                    state = validate_state(sample)
                    last_state = tick
            if tick - last_state > 0.25:
                if not stale:
                    print('Robot state timed out; disarming into damping. Repeat the startup sequence to resume.', flush=True)
                stale = True
                controller.disarm()
            elif tick >= next_control:
                stale = False
                controller.update(state, tick)
                next_control = tick + CONTROL_DT
            controller.command.crc = command_crc(controller.command)
            writer.write(controller.command)
            time.sleep(max(0, COMMUNICATION_DT - (time.monotonic() - tick)))
    finally:
        command = controller.disarm()
        command.crc = command_crc(command)
        for _ in range(20):
            try:
                writer.write(command)
            except Exception as error:
                print(f'Could not send shutdown damping: {error}', flush=True)
                break
            time.sleep(COMMUNICATION_DT)


def main(argv=None):
    args = parse_args(argv)
    # Load the model before making any real-robot control-mode changes.
    controller = PolicyController(args.policy, real=args.real)
    dds = Go2DDS(domain_id=args.domain, interface=args.interface)
    reader = dds.reader('rt/lowstate', LowState_)
    print(f'Waiting for {"real Go2" if args.real else "MuJoCo"} on domain {args.domain} / {args.interface}...', flush=True)
    state = wait_for_state(reader)
    if args.real:
        print('Real-robot mode: keep Go2 supported, with its physical stop control available. Requesting StandDown and mode release.', flush=True)
        release_motion_control(dds)
        reader.take(32)  # Discard samples queued during the mode handover.
        state = wait_for_state(reader)
        # Already-held buttons must be released and pressed again to start.
        controller.previous_keys = int.from_bytes(bytes(state.wireless_remote)[2:4], 'little')
        print('Go2 remote: L1+Y, L1+R1, wait 2s, L1+R2, wait 2s, L1+A. L1+Y returns to damping.', flush=True)
    else:
        print('Focus the MuJoCo keyboard window: 1, 2, wait 2s, 3, wait 2s, 4.', flush=True)
    writer = dds.writer('rt/lowcmd', LowCmd_)
    run_controller(controller, reader, writer, state)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        print('Stopped.')
    except (RuntimeError, ValueError) as error:
        raise SystemExit(str(error))
