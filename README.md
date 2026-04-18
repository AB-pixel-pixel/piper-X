# robotics_arm（Piper X + MuJoCo 纯仿真）

本项目面向初学者，目标是跑通并理解 Piper X 机械臂在 MuJoCo 里的最小闭环：模型 → 可控（actuator） → 任务/演示。

## 0. 环境要求

- Linux/WSL2（Ubuntu 推荐）
- Python 3（项目默认在 Python 3.8 上验证）
- 已安装依赖见 [requirements.txt](file:///home/ling/project/robotics_arm/requirements.txt)

## 1. 安装 Python 依赖

在仓库根目录执行：

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -U pip setuptools wheel
python -m pip install -r requirements.txt
```

或一键安装并做最小自检：

```bash
bash scripts/bootstrap_python.sh
```

验证 MuJoCo Python 版可用（不需要开窗）：

```bash
source .venv/bin/activate
python scripts/test_mujoco.py
```

## 2. 准备外部资源（URDF / mesh / SDK）

本项目会把外部仓库放在 `repos/` 下（默认已在 `.gitignore` 里忽略）。

### 2.1 下载 agx_arm_ros（含 URDF 子模块）

```bash
mkdir -p repos
cd repos
git clone https://github.com/agilexrobotics/agx_arm_ros.git
cd agx_arm_ros
git submodule update --init --recursive
cd ../..
```

### 2.2（可选）下载并可编辑安装 pyAgxArm

```bash
mkdir -p repos
cd repos
git clone https://github.com/agilexrobotics/pyAgxArm.git
cd pyAgxArm
python -m pip install -e .
cd ../..
```

## 3. 生成可加载/可控制的模型（URDF → MJCF）

注意：从 URDF 直接导入 MuJoCo 通常没有 actuator（`nu=0`），因此需要额外生成带 motor 的 MJCF。

### 3.1 生成 MuJoCo 可加载的 URDF（复制 mesh 并重写路径）

```bash
source .venv/bin/activate
python scripts/prepare_urdf_for_mujoco.py --arm piper_x
```

产物在：

- `models/agx_arm_ros_urdf/piper_x/piper_x_description.urdf`
- 以及同目录下复制出来的 mesh 文件

### 3.2 导出 MJCF 并添加 motor actuator

```bash
python scripts/build_mjcf_with_actuators.py --arm piper_x
```

产物在：

- `models/agx_arm_ros_urdf/piper_x/piper_x_mujoco.xml`

## 4. 运行示例

### 4.1 最小样例（关节驱动验证）

```bash
source .venv/bin/activate
python scripts/minimal_piper_x_demo.py
```

### 4.2 可视化演示（场景 + “捡起方块”）

```bash
source .venv/bin/activate
python scripts/build_piper_x_scene.py
python scripts/visual_piper_x_pick_demo.py
```

如果 GUI 开窗失败，脚本会输出离屏渲染帧到 `out/`（已在 `.gitignore` 中忽略）。

## 5. MuJoCo 二进制（可选）

如需下载/重下 MuJoCo 官方二进制包到 `third_party/`：

```bash
bash scripts/download_mujoco_bin.sh 2.3.7
```

## 6. 学习路线

建议按 [LEARNING_PLAN_PIPER.md](file:///home/ling/project/robotics_arm/LEARNING_PLAN_PIPER.md) 从仿真与关节空间控制开始。  
关键概念提示：`model.nq/nv/nu`、`data.qpos/qvel/ctrl`、`mujoco.mj_step`。
