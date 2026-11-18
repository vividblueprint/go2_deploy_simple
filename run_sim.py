"""Launch the existing keyboard-enabled simulator using this workspace's environment."""
from importlib.metadata import version
from pathlib import Path
import runpy
import sys

ROOT = Path(__file__).resolve().parent

if __name__ == '__main__':
    if version('cyclonedds') != '0.10.2':
        raise SystemExit('The existing simulator bridge needs the 0.10.2 environment. Select the SDK 0.10.2 branch and run uv sync --extra sim.')
    simulator = ROOT.parent / 'unitree_mujoco' / 'simulate_python'
    original_sdk = ROOT.parent / 'unitree_sdk2_python'
    if not (simulator / 'unitree_mujoco.py').is_file() or not (original_sdk / 'unitree_sdk2py').is_dir():
        raise SystemExit('Expected sibling unitree_mujoco and unitree_sdk2_python source folders.')
    # Only the existing simulator bridge uses the original SDK; deploy.py uses Go2DDS.
    sys.path[:0] = [str(simulator), str(original_sdk)]
    runpy.run_path(str(simulator / 'unitree_mujoco.py'), run_name='__main__')
