from __future__ import annotations

import math
import os
from pathlib import Path
import sys

import mujoco
import numpy as np


def _write_ppm(path: Path, rgb: np.ndarray) -> None:
    h, w, c = rgb.shape
    if c != 3:
        raise ValueError("Expected RGB image")
    header = f"P6\n{w} {h}\n255\n".encode("ascii")
    with path.open("wb") as f:
        f.write(header)
        f.write(rgb.tobytes())


def _ensure_models(repo_root: Path) -> Path:
    urdf = repo_root / "models" / "agx_arm_ros_urdf" / "piper_x" / "piper_x_description.urdf"
    mjcf = repo_root / "models" / "agx_arm_ros_urdf" / "piper_x" / "piper_x_mujoco.xml"
    scene = repo_root / "models" / "agx_arm_ros_urdf" / "piper_x" / "piper_x_scene.xml"

    if not urdf.exists():
        raise FileNotFoundError(
            f"缺少：{urdf}\n"
            "请先运行：python scripts/prepare_urdf_for_mujoco.py --arm piper_x"
        )
    if not mjcf.exists():
        raise FileNotFoundError(
            f"缺少：{mjcf}\n"
            "请先运行：python scripts/build_mjcf_with_actuators.py --arm piper_x"
        )
    if not scene.exists():
        raise FileNotFoundError(
            f"缺少：{scene}\n"
            "请先运行：python scripts/build_piper_x_scene.py"
        )
    return scene


def _q_traj(t: float, nq: int) -> np.ndarray:
    q = np.zeros((nq,), dtype=float)
    q[0] = 0.25 * math.sin(2.0 * math.pi * 0.20 * t)
    if nq > 2:
        q[1] = 0.20 * math.sin(2.0 * math.pi * 0.20 * t + math.pi / 3.0)
        q[2] = 0.20 * math.sin(2.0 * math.pi * 0.20 * t + 2.0 * math.pi / 3.0)
    if nq > 4:
        q[3] = 0.15 * math.sin(2.0 * math.pi * 0.20 * t)
        q[4] = 0.10 * math.sin(2.0 * math.pi * 0.20 * t)
    return q


def _pd_ctrl(q: np.ndarray, qvel: np.ndarray, q_des: np.ndarray, kp: float, kd: float) -> np.ndarray:
    return kp * (q_des - q) - kd * qvel


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    scene_path = _ensure_models(repo_root)

    model = mujoco.MjModel.from_xml_path(str(scene_path))
    data = mujoco.MjData(model)

    arm_joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    arm_qpos_adr: list[int] = []
    arm_dof_adr: list[int] = []
    for j in arm_joint_names:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, j)
        arm_qpos_adr.append(int(model.jnt_qposadr[jid]))
        arm_dof_adr.append(int(model.jnt_dofadr[jid]))

    ee_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    box_joint = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "box_free")
    box_qpos_adr = int(model.jnt_qposadr[box_joint])

    kp = 10.0
    kd = 1.0

    attached = False

    def step_once() -> None:
        nonlocal attached

        t = float(data.time)
        q = np.array([data.qpos[a] for a in arm_qpos_adr], dtype=float)
        qvel = np.array([data.qvel[a] for a in arm_dof_adr], dtype=float)
        q_des = _q_traj(t, len(arm_qpos_adr))
        u = _pd_ctrl(q, qvel, q_des, kp=kp, kd=kd)
        u = np.clip(u, -1.0, 1.0)
        data.ctrl[: len(u)] = u

        if (not attached) and t > 1.0:
            attached = True

        if attached:
            pos = np.array(data.xpos[ee_body]).copy()
            quat = np.array(data.xquat[ee_body]).copy()
            data.qpos[box_qpos_adr : box_qpos_adr + 3] = pos
            data.qpos[box_qpos_adr + 3 : box_qpos_adr + 7] = quat
            data.qvel[model.jnt_dofadr[box_joint] : model.jnt_dofadr[box_joint] + 6] = 0.0

        mujoco.mj_step(model, data)

    use_viewer = os.environ.get("MUJOCO_OFFSCREEN", "0") != "1"
    if use_viewer:
        try:
            from mujoco import viewer as mjviewer

            with mjviewer.launch_passive(model, data) as viewer:
                viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FIXED
                viewer.cam.fixedcamid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cam_fixed")
                while viewer.is_running():
                    step_once()
                    viewer.sync()
            return
        except Exception as e:
            print("viewer_failed:", repr(e))
            print("fall_back_to_offscreen=1")

    out_dir = repo_root / "out" / "piper_x_pick_demo_frames"
    out_dir.mkdir(parents=True, exist_ok=True)

    renderer = mujoco.Renderer(model, width=640, height=480)
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, "cam_fixed")

    n_frames = 120
    for i in range(n_frames):
        for _ in range(5):
            step_once()
        renderer.update_scene(data, camera=cam_id)
        rgb = renderer.render()
        _write_ppm(out_dir / f"frame_{i:04d}.ppm", rgb)

    print("frames_written", str(out_dir), "count", n_frames)
    print("hint: use an image viewer that supports PPM, or convert with ffmpeg if available.")


if __name__ == "__main__":
    sys.exit(main())
