"""Real-mode startup and stopping logic; all Ethernet/RPC operations are mocked."""
from contextlib import redirect_stderr
from io import StringIO
import struct
import unittest
from unittest.mock import Mock, patch

import numpy as np
from unitree_sdk2_go2 import LowState_, new_message

import deploy
from policy import PolicyController, DEFAULT_ANGLES, DDS_JOINT_MIN, DDS_JOINT_MAX


def state(keys=0):
    message = new_message(LowState_)
    message.imu_state.quaternion = [1., 0., 0., 0.]
    for i, motor in enumerate(message.motor_state[:12]):
        motor.q = float(DEFAULT_ANGLES[i])
    remote = bytearray(40)
    struct.pack_into('<H', remote, 2, keys)
    message.wireless_remote = list(remote)
    return message


class ArgumentTests(unittest.TestCase):
    def test_default_is_mujoco(self):
        args = deploy.parse_args([])
        self.assertFalse(args.real)
        self.assertEqual((args.domain, args.interface), (1, 'lo'))

    def test_real_uses_named_interface_and_domain_zero(self):
        with patch('deploy.socket.if_nametoindex', return_value=2):
            args = deploy.parse_args(['--real', '--interface', 'enp3s0', '--policy', 'ee'])
        self.assertTrue(args.real)
        self.assertEqual((args.domain, args.interface, args.policy), (0, 'enp3s0', 'ee'))

    def test_bad_interface_arguments_fail_before_creating_dds(self):
        for argv in (['--real'], ['--real', '--interface', 'lo'], ['--interface', 'enp3s0']):
            with self.subTest(argv=argv), redirect_stderr(StringIO()), self.assertRaises(SystemExit):
                deploy.parse_args(argv)
        with patch('deploy.socket.if_nametoindex', side_effect=OSError), redirect_stderr(StringIO()), self.assertRaises(SystemExit):
            deploy.parse_args(['--real', '--interface', 'missing0'])


class HandoverTests(unittest.TestCase):
    @patch('deploy.time.sleep')
    @patch('deploy.SportClient')
    @patch('deploy.MotionSwitcherClient')
    def test_active_mode_is_stood_down_released_and_rechecked(self, switch_class, sport_class, sleep):
        switch = switch_class.return_value
        switch.CheckMode.side_effect = [(0, {'name': 'normal'}), (0, {'name': ''})]
        switch.ReleaseMode.return_value = (0, None)
        sport_class.return_value.StandDown.return_value = 0
        deploy.release_motion_control(Mock())
        sport_class.return_value.StandDown.assert_called_once()
        switch.ReleaseMode.assert_called_once()
        self.assertEqual(switch.CheckMode.call_count, 2)

    @patch('deploy.SportClient')
    @patch('deploy.MotionSwitcherClient')
    def test_inactive_mode_needs_no_motion_command(self, switch_class, sport_class):
        switch_class.return_value.CheckMode.return_value = (0, {'name': ''})
        deploy.release_motion_control(Mock())
        sport_class.assert_not_called()
        switch_class.return_value.ReleaseMode.assert_not_called()

    @patch('deploy.time.sleep')
    @patch('deploy.SportClient')
    @patch('deploy.MotionSwitcherClient')
    def test_failed_rpc_or_persistent_mode_aborts(self, switch_class, sport_class, sleep):
        switch = switch_class.return_value
        for response in ((3104, None), (0, {}), (0, {'name': None})):
            switch.CheckMode.return_value = response
            with self.assertRaises(RuntimeError):
                deploy.release_motion_control(Mock())
        switch.CheckMode.return_value = (0, {'name': 'normal'})
        sport_class.return_value.StandDown.return_value = 1
        with self.assertRaises(RuntimeError):
            deploy.release_motion_control(Mock())
        sport_class.return_value.StandDown.return_value = 0
        switch.ReleaseMode.return_value = (3104, None)
        with self.assertRaises(RuntimeError):
            deploy.release_motion_control(Mock())
        switch.ReleaseMode.return_value = (0, None)
        with self.assertRaisesRegex(RuntimeError, 'still active'):
            deploy.release_motion_control(Mock())

    @patch('deploy.PolicyController')
    @patch('deploy.Go2DDS')
    @patch('deploy.release_motion_control', side_effect=RuntimeError('mode release failed'))
    @patch('deploy.socket.if_nametoindex', return_value=2)
    def test_no_motor_writer_is_created_when_handover_fails(self, interface, release, dds_class, controller):
        dds_class.return_value.reader.return_value.take_one.return_value = state()
        with self.assertRaisesRegex(RuntimeError, 'mode release failed'):
            deploy.main(['--real', '--interface', 'enp3s0'])
        dds_class.return_value.writer.assert_not_called()

    @patch('deploy.PolicyController')
    @patch('deploy.Go2DDS')
    @patch('deploy.release_motion_control')
    @patch('deploy.socket.if_nametoindex', return_value=2)
    def test_invalid_initial_state_aborts_before_handover(self, interface, release, dds_class, controller):
        bad = state()
        bad.motor_state[0].q = float('nan')
        dds_class.return_value.reader.return_value.take_one.return_value = bad
        with self.assertRaises(RuntimeError):
            deploy.main(['--real', '--interface', 'enp3s0'])
        release.assert_not_called()
        dds_class.return_value.writer.assert_not_called()

    @patch('deploy.run_controller')
    @patch('deploy.PolicyController')
    @patch('deploy.Go2DDS')
    @patch('deploy.release_motion_control')
    @patch('deploy.socket.if_nametoindex', return_value=2)
    def test_real_handover_precedes_motor_writer_and_seeds_held_buttons(self, interface, release, dds_class, controller_class, run):
        events = []
        held = (1 << 1) | (1 << 11)
        fresh = state(held)
        reader = dds_class.return_value.reader.return_value
        reader.take_one.return_value = fresh
        release.side_effect = lambda dds: events.append('release')
        dds_class.return_value.writer.side_effect = lambda *args: events.append('writer') or Mock()
        deploy.main(['--real', '--interface', 'enp3s0'])
        self.assertEqual(events, ['release', 'writer'])
        self.assertEqual(reader.take_one.call_count, 2)
        self.assertEqual(controller_class.return_value.previous_keys, held)
        controller_class.assert_called_once_with('ts', real=True)
        dds_class.assert_called_once_with(domain_id=0, interface='enp3s0')
        run.assert_called_once()

    @patch('deploy.run_controller')
    @patch('deploy.PolicyController')
    @patch('deploy.Go2DDS')
    @patch('deploy.release_motion_control')
    def test_mujoco_never_calls_robot_mode_services(self, release, dds_class, controller, run):
        dds_class.return_value.reader.return_value.take_one.return_value = state()
        deploy.main([])
        release.assert_not_called()
        dds_class.assert_called_once_with(domain_id=1, interface='lo')


class StateAndPolicyTests(unittest.TestCase):
    def test_invalid_joint_imu_and_remote_data_are_rejected(self):
        for kind in ('joint', 'quaternion', 'remote'):
            message = state()
            if kind == 'joint':
                message.motor_state[0].q = float('nan')
            elif kind == 'quaternion':
                message.imu_state.quaternion = [0., 0., 0., 0.]
            else:
                message.wireless_remote = [0] * 3
            with self.subTest(kind=kind), self.assertRaises((RuntimeError, ValueError)):
                deploy.validate_state(message)

    def test_disarmed_real_controller_requires_fresh_damping_button(self):
        controller = PolicyController('ts', real=True)
        controller.disarm()
        controller.update(state((1 << 1) | (1 << 0)), 0)
        self.assertEqual(controller.state, 'damping')
        self.assertFalse(controller.armed)
        controller.update(state(), .1)
        controller.update(state((1 << 1) | (1 << 11)), .2)
        self.assertTrue(controller.armed)
        controller.update(state(), .3)
        controller.update(state((1 << 1) | (1 << 0)), .4)
        self.assertEqual(controller.state, 'sit')
        controller.disarm()
        self.assertFalse(controller.armed)

    def test_original_remote_zero_torque_combination(self):
        controller = PolicyController('ts', real=True)
        controller.damping()
        controller.update(state((1 << 1) | (1 << 10)), 0)
        self.assertEqual(controller.state, 'zero')
        self.assertFalse(controller.armed)
        self.assertTrue(all(m.kp == 0 and m.kd == 0 and m.tau == 0 for m in controller.command.motor_cmd))

    def test_real_profile_limits_targets_and_scales_remote_input(self):
        controller = PolicyController('ts', real=True)
        controller.state, controller.armed = 'walk', True
        controller._infer = Mock(return_value=np.full(12, 100.))
        message = state()
        raw = bytearray(message.wireless_remote)
        struct.pack_into('<f', raw, 20, 1.)
        message.wireless_remote = list(raw)
        command = controller.update(message, 0)
        np.testing.assert_allclose(controller._infer.call_args.args[1], [.25, 0, 0])
        targets = np.array([m.q for m in command.motor_cmd[:12]])
        self.assertTrue((targets >= DDS_JOINT_MIN).all())
        self.assertTrue((targets <= DDS_JOINT_MAX).all())
        self.assertEqual(controller.stand_gains, (60., 5.))
        self.assertEqual(controller.damping_kd, 4.)

    @patch('deploy.time.sleep')
    def test_invalid_running_state_sends_shutdown_damping(self, sleep):
        controller = PolicyController('ts', real=True)
        controller.state, controller.armed = 'walk', True
        bad = state()
        bad.motor_state[0].dq = float('nan')
        reader, writer = Mock(), Mock()
        reader.take.return_value = [bad]
        with self.assertRaises(RuntimeError):
            deploy.run_controller(controller, reader, writer, state())
        self.assertEqual(writer.write.call_count, 20)
        self.assertFalse(controller.armed)
        self.assertTrue(all(m.kp == 0 and m.kd == 4 for m in controller.command.motor_cmd))

    @patch('deploy.time.sleep')
    @patch('deploy.time.monotonic', side_effect=[0., .3, .3, .32])
    def test_stale_state_disarms_and_recovery_does_not_resume_walk(self, clock, sleep):
        controller = PolicyController('ts', real=True)
        controller.state, controller.armed = 'walk', True
        reader, writer = Mock(), Mock()
        reader.take.side_effect = [[], [state((1 << 1) | (1 << 0))]]
        calls = 0
        def write(command):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise KeyboardInterrupt
        writer.write.side_effect = write
        with self.assertRaises(KeyboardInterrupt):
            deploy.run_controller(controller, reader, writer, state())
        self.assertEqual(controller.state, 'damping')
        self.assertFalse(controller.armed)
        self.assertGreaterEqual(writer.write.call_count, 22)


if __name__ == '__main__':
    unittest.main()
