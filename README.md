# English | [中文](README_cn.md)

# humanoid-mujoco-sim

## 1. Run the Simulation

- Operating Environment: Python 3.8 or higher is recommended.

- Open a Bash terminal.

- Download the MuJoCo simulator code:

  ```
  git clone --recurse git@github.com:limxdynamics/humanoid-mujoco-sim.git
  ```

- Install the motion control development library:

  - For Linux x86_64 environment:

    ```
    pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/amd64/limxsdk-*-py3-none-any.whl
    ```

  - For Linux aarch64 environment:

    ```
    pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/aarch64/limxsdk-*-py3-none-any.whl
    ```

- Set the robot type:

  - List available robot types with the Shell command `tree -L 3 -P "meshes" -I "urdf|world|xml|usd" humanoid-mujoco-sim/humanoid-description`：

    ```
    limx@limx:~$ tree -L 3 -P "meshes" -I "urdf|world|xml|usd" humanoid-mujoco-sim/humanoid-description
    humanoid-mujoco-sim/humanoid-description
    ├── HU_D03_description
    │   └── meshes
    │       └── HU_D03_03
    └── HU_D04_description
        └── meshes
            └── HU_D04_01
    ```
    
  - Taking `HU_D04_01` (replace with your actual robot type) as an example, set the robot model type:
  
    ```
    echo 'export ROBOT_TYPE=HU_D04_01' >> ~/.bashrc && source ~/.bashrc
    ```
  
- Run the MuJoCo simulator:

  ```
  python humanoid-mujoco-sim/simulator.py
  ```

## 2. Simulation Demonstration

![](doc/simulator.gif)
