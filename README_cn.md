# 中文 | [English](README.md)

# humanoid-mujoco-sim

## 1. 运行仿真

- 运行环境：推荐Pyhon 3.8 及以上版本

- 打开一个 Bash 终端。

- 下载 MuJoCo 仿真器代码：

  ```
  git clone --recurse git@github.com:limxdynamics/humanoid-mujoco-sim.git
  ```

- 安装运动控制开发库：

  - Linux x86_64 环境

    ```
    pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/amd64/limxsdk-*-py3-none-any.whl
    ```

  - Linux aarch64 环境

    ```
    pip install humanoid-mujoco-sim/limxsdk-lowlevel/python3/aarch64/limxsdk-*-py3-none-any.whl
    ```

- 设置机器人类型

  - 通过 Shell 命令 `tree -L 3 -P "meshes" -I "urdf|world|xml|usd" humanoid-mujoco-sim/humanoid-description` 列出可用的机器人类型：

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
    
  - 以`HU_D04_01`（请根据实际机器人类型进行替换）为例，设置机器人型号类型：
  
    ```
    echo 'export ROBOT_TYPE=HU_D04_01' >> ~/.bashrc && source ~/.bashrc
    ```
  
- 运行 MuJoCo 仿真器：

  ```
  python humanoid-mujoco-sim/simulator.py
  ```

## 2. 仿真展示

![](doc/simulator.gif)
