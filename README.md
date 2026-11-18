
<div align="center">

<h1 align="center" >

<img width="25px" src="https://em-content.zobj.net/source/microsoft-teams/337/heart-with-ribbon_1f49d.png"/>
<img width="25px" src="https://em-content.zobj.net/source/microsoft-teams/337/heart-with-ribbon_1f49d.png"/>
<img width="25px" src="https://em-content.zobj.net/source/microsoft-teams/337/heart-with-ribbon_1f49d.png"/>
<b>GO2 Simple Deploy sim2sim/sim2real</b>
<img width="25px" src="https://em-content.zobj.net/source/microsoft-teams/337/heart-with-ribbon_1f49d.png"/>
<img width="25px" src="https://em-content.zobj.net/source/microsoft-teams/337/heart-with-ribbon_1f49d.png"/>
<img width="25px" src="https://em-content.zobj.net/source/microsoft-teams/337/heart-with-ribbon_1f49d.png"/>

</h1>

[![GitHub](https://img.shields.io/badge/GitHub-vividblueprint-181717?logo=github&logoColor=white)](https://github.com/vividblueprint)
[![Issues](https://img.shields.io/github/issues/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple/issues)
[![Pull Requests](https://img.shields.io/github/issues-pr/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple/pulls)
[![Contributors](https://img.shields.io/github/contributors/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple/graphs/contributors)
[![Stars](https://img.shields.io/github/stars/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple/stargazers)
[![Forks](https://img.shields.io/github/forks/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple/forks)
[![Watchers](https://img.shields.io/github/watchers/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple/watchers)
[![Last Commit](https://img.shields.io/github/last-commit/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple/commits)
[![Monthly Commits](https://img.shields.io/github/commit-activity/m/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple/commits)
[![Languages](https://img.shields.io/github/languages/count/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple)
[![Code Size](https://img.shields.io/github/languages/code-size/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple)
[![Repo Size](https://img.shields.io/github/repo-size/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple)
[![Total Commits](https://img.shields.io/github/commit-activity/t/vividblueprint/go2_deploy_simple?style=flat&label=Total%20Commits)](https://github.com/vividblueprint/go2_deploy_simple/commits)
[![Files](https://img.shields.io/github/directory-file-count/vividblueprint/go2_deploy_simple?type=file&style=flat)](https://github.com/vividblueprint/go2_deploy_simple/tree/HEAD)
[![License](https://img.shields.io/github/license/vividblueprint/go2_deploy_simple?style=flat)](https://github.com/vividblueprint/go2_deploy_simple/blob/HEAD/LICENSE)

</div>

# go2_deploy_simple

Run the original TS, EE, and WaQ `.pt` policies with `unitree_sdk2_go2` in MuJoCo or on a real Go2.
Uses **Python 3.11** and **CycloneDDS 11.1.0**. Model weights are unchanged.

## Install

Requires [uv](https://docs.astral.sh/uv/getting-started/installation/), the sibling
`unitree_sdk2_go2` source folder, and the existing files in `models/`.

```bash
# make a directory
mkdir ~/unitree_go2
cd ~/unitree_go2

# clone repos
git clone https://github.com/vividblueprint/go2_mujoco.git
git clone https://github.com/vividblueprint/unitree_sdk2_go2.git
git clone https://github.com/vividblueprint/go2_deploy_simple.git

cd ~/unitree_go2/go2_deploy_simple
uv sync
```

CycloneDDS includes its native runtime. No external DDS installation, shared
script, application `.toml`, or lockfile is needed. The SDK does not need its own
separate environment.

## MuJoCo

Start the installed simulator in terminal 1:

```bash
cd ~/unitree_go2/go2_deploy_simple
uv run ../go2_mujoco/simulate.py
```

Start the policy in terminal 2:

```bash
cd ~/unitree_go2/go2_deploy_simple
uv run deploy.py --policy ts
```

Choose `--policy ee` or `--policy waq` for the other models. Stop other motor
controllers before starting this one. Both processes use domain **1** on **lo**.
Simulation is the default; it does not call robot motion services.

### Keyboard

Focus the simulator's keyboard window:

1. **1**: damping.
2. **2**: sit; wait **2 seconds**.
3. **3**: stand; wait **2 seconds**.
4. **4**: enable walking.

Hold **WASD** to move and **Q/E** to turn. **Space** stops movement; **1** returns
to damping. Ctrl+C stops deployment and sends brief damping commands.

## Real Go2 over Ethernet

Connect the computer to a Go2 that supports SDK low-level control. Configure the
Ethernet adapter on the robot's subnet using your robot's network settings.
Find your adapter name with `ip -br address`, then replace `enp3s0` below:

```bash
cd ~/unitree_go2/go2_deploy_simple
uv run deploy.py --real --interface enp3s0 --policy ts
```

This uses DDS domain **0** on the selected adapter. Do not run another low-level
motor controller at the same time. The program waits for valid robot state,
requests **StandDown** if the built-in motion controller is active, and checks
that it has released control before publishing motor commands. Startup can make
the robot lie down. Keep it supported and its physical stop control available
for the first trials; real hardware has not been tested with this runner.

Use the **Go2 wireless remote**, with fresh button presses after startup:

1. **L1+Y**: damping and enable the startup sequence.
2. **L1+R1**: sit; wait **2 seconds**.
3. **L1+R2**: stand; wait **2 seconds**.
4. **L1+A**: enable the walking policy; use the joysticks to move and turn.

**L1+Y** returns to damping from any state. From damping, **L1+X** selects zero
torque. MuJoCo's keyboard window controls simulation only.

Real mode uses the original deployment's standing gains (`kp=60`, `kd=5`) and
damping gain (`kd=4`). Joystick commands are limited to **25%** of the original
range, and joint targets are clipped to the bundled Go2 model's joint limits.
These limits do not establish hardware safety; verify the model, gains, and
limits for your robot before enabling walking.

State loss for **0.25 seconds** disarms into damping; repeat the startup sequence
after state reception recovers. Ctrl+C attempts brief damping commands. A
process crash or disconnected Ethernet cannot guarantee stopping the robot.
Exiting does not automatically restore the built-in motion controller.

## Models

| Policy | File | History |
| --- | --- | --- |
| TS | `policy_ts_gs.pt` | 20 frames |
| EE | `Dec15_14-57-47_gym_ee_policy.pt` | 10 frames |
| WaQ | `Dec16_09-19-17_gym_dreamwaq_policy.pt` | 20 frames |

Observation scaling, history, and joint ordering match the original controller.
Control runs at 50 Hz and commands publish at approximately 500 Hz. State loss
for 0.25 seconds triggers damping. MuJoCo startup gains are `kp=40`, `kd=1`; policy gains
are `kp=20`, `kd=0.5` in both modes. `models/SHA256.json` records the copied model hashes.

## Test

The comparison tests need the original deployment source at its relocated path:

```bash
cd ~/unitree_go2/go2_deploy_simple
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git

GO2_ORIGINAL_DEPLOY=~/unitree_go2/go2_deploy_python \
  .venv/bin/python -m unittest discover -s tests -v
```

The original source is needed only by the model comparison tests, not the
deployment runner. Real-mode tests mock all Ethernet and RPC operations; they
do not contact a robot. The original deployment and model files are unchanged.
