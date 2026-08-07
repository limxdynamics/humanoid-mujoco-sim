# humanoid-mujoco-sim

MuJoCo simulation environment for LimX humanoid robots (HU_D03, HU_D04). Use it to visualize robot models, test motion controllers, and validate RL policies in simulation before deploying to hardware.

## 1. Quick Start

### Prerequisites

- Python 3.8 or higher
- Linux (x86_64)

> The bundled `prebuild/kinematic_projection` helper is x86_64-only. aarch64 users can install the limxsdk wheel but simulation functionality will be limited.

### Install

```bash
# Clone with submodules (HTTPS)
git clone --recurse-submodules https://github.com/limxdynamics/humanoid-mujoco-sim.git
cd humanoid-mujoco-sim

# Install dependencies
pip install -r requirements.txt

# Install the LimX motion control SDK
pip install limxsdk-lowlevel/python3/amd64/limxsdk-*-py3-none-any.whl
```

### Set Robot Type

Available models: `HU_D03_03`, `HU_D04_01`

```bash
echo 'export ROBOT_TYPE=HU_D04_01' >> ~/.bashrc && source ~/.bashrc
```

### Run

```bash
python simulator.py
```

You should see the robot standing in a MuJoCo window. Use mouse (right-click + drag) to rotate the view, scroll to zoom.

## 2. Simulation Controls

| Key | Action |
|-----|--------|
| `Tab` | Toggle left/right UI panels |
| `Space` | Pause / Resume simulation |
| Right-click + drag | Rotate camera |
| Scroll | Zoom |

## 3. Virtual Joystick

Use the included virtual joystick to control the robot interactively:

```bash
# Linux
./robot-joystick/robot-joystick
# Windows
robot-joystick/robot-joystick.exe
```

## 4. Simulation Demonstration

![simulator](doc/simulator.gif)

## 5. Related Repositories

- [humanoid-description](https://github.com/limxdynamics/humanoid-description) — Robot model files (URDF/MuJoCo)
- [humanoid-rl-deploy-python](https://github.com/limxdynamics/humanoid-rl-deploy-python) — Python RL policy deployment
- [humanoid-rl-deploy-cpp](https://github.com/limxdynamics/humanoid-rl-deploy-cpp) — ROS-free C++ RL deployment
- [humanoid-rl-isaaclab](https://github.com/limxdynamics/humanoid-rl-isaaclab) — RL training with Isaac Lab
