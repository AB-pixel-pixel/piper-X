from __future__ import annotations

import math
from pathlib import Path

import mujoco
import numpy as np


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    mjcf_path = repo_root / "models" / "agx_arm_ros_urdf" / "piper_x" / "piper_x_mujoco.xml"
    if not mjcf_path.exists():
        raise FileNotFoundError(
            f"找不到 MJCF：{mjcf_path}\n"
            "请先运行：\n"
            "  source .venv/bin/activate\n"
            "  python scripts/prepare_urdf_for_mujoco.py --arm piper_x\n"
            "  python scripts/build_mjcf_with_actuators.py --arm piper_x"
        )

    model = mujoco.MjModel.from_xml_path(str(mjcf_path))
    data = mujoco.MjData(model)

    dt = float(model.opt.timestep)
    steps = int(2.0 / dt)

    amp = 0.2
    freq_hz = 0.5

    for i in range(steps):
        t = float(data.time)
        if model.nu > 0:
            u = amp * math.sin(2.0 * math.pi * freq_hz * t)
            data.ctrl[:] = 0.0
            data.ctrl[0] = u

        mujoco.mj_step(model, data)

        if i % max(1, int(0.1 / dt)) == 0:
            qpos = np.array(data.qpos).copy()
            qvel = np.array(data.qvel).copy()
            ctrl = np.array(data.ctrl).copy() if model.nu > 0 else np.array([])
            print(
                "t",
                f"{data.time:.3f}",
                "qpos0",
                f"{qpos[0]: .4f}",
                "qvel0",
                f"{qvel[0]: .4f}",
                "ctrl0",
                f"{(ctrl[0] if ctrl.size else 0.0): .4f}",
            )

    print("done", "steps", steps, "dt", dt, "nq", model.nq, "nu", model.nu)


if __name__ == "__main__":
    main()
