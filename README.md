本目录用于在 WSL2 上部署 MuJoCo 仿真与后续机械臂科研代码。

已完成
- 创建 Python 虚拟环境：.venv
- 安装 MuJoCo Python 版：mujoco==2.3.7（可直接在 Python 中加载/步进仿真）
- 下载 MuJoCo 二进制发行版：third_party/mujoco-2.3.7（含 simulate 等示例程序）
- 下载并安装机械臂 SDK（可编辑安装）：repos/pyAgxArm（pyAgxArm==1.0.0）
- 下载 ROS 仓库（含 URDF 模型引用）：repos/agx_arm_ros（并已拉取其 submodule：agx_arm_urdf）
- 生成可被 MuJoCo 直接加载的 Piper URDF：models/agx_arm_ros_urdf/piper/piper_description.urdf

快速验证
- 进入本目录后执行：
  - source .venv/bin/activate
  - python scripts/test_mujoco.py

URDF -> MuJoCo（Piper / Nero）
- 生成可加载的 URDF（会把 mesh 复制到 models/agx_arm_ros_urdf 目录并重写路径）：
  - source .venv/bin/activate
  - python scripts/prepare_urdf_for_mujoco.py --arm piper
- 用 MuJoCo 加载验证：
  - python -c "import mujoco; mujoco.MjModel.from_xml_path('models/agx_arm_ros_urdf/piper/piper_description.urdf'); print('ok')"

学习路线
- 初学者建议从仿真开始：LEARNING_PLAN_PIPER.md

MuJoCo 二进制（可选）
- 如需下载/重下：
  - bash scripts/download_mujoco_bin.sh 2.3.7

依赖管理
- requirements.txt 记录当前可用的 Python 依赖版本
- 如需在新环境复现：
  - python3 -m venv .venv
  - source .venv/bin/activate
  - python -m pip install -U pip setuptools wheel
  - python -m pip install -r requirements.txt

代码库下载
- 当前已下载：repos/pyAgxArm（来源：https://github.com/agilexrobotics/pyAgxArm.git）
- 如需再下载其他仓库：设置环境变量 REPO_URL 后运行：
  - bash scripts/clone_repo.sh
