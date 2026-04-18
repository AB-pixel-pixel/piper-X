from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np


def ensure_child(parent: ET.Element, tag: str) -> ET.Element:
    child = parent.find(tag)
    if child is None:
        child = ET.SubElement(parent, tag)
    return child


def main() -> None:
    repo_root = Path(__file__).resolve().parents[1]
    base_mjcf = repo_root / "models" / "agx_arm_ros_urdf" / "piper_x" / "piper_x_mujoco.xml"
    if not base_mjcf.exists():
        raise FileNotFoundError(
            f"找不到基础 MJCF：{base_mjcf}\n"
            "请先运行：python scripts/build_mjcf_with_actuators.py --arm piper_x"
        )

    model = mujoco.MjModel.from_xml_path(str(base_mjcf))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    ee_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
    ee_pos = np.array(data.xpos[ee_body]).copy()

    box_pos = ee_pos + np.array([0.05, 0.00, -0.12])
    if box_pos[2] < 0.05:
        box_pos[2] = 0.05

    cam_pos = ee_pos + np.array([0.60, 0.25, 0.30])
    target = ee_pos
    forward = target - cam_pos
    forward_norm = float(np.linalg.norm(forward))
    if forward_norm < 1e-9:
        forward = np.array([-1.0, 0.0, 0.0])
    else:
        forward = forward / forward_norm
    z_axis = -forward
    up = np.array([0.0, 0.0, 1.0])
    x_axis = np.cross(up, z_axis)
    x_norm = float(np.linalg.norm(x_axis))
    if x_norm < 1e-9:
        up = np.array([0.0, 1.0, 0.0])
        x_axis = np.cross(up, z_axis)
        x_norm = float(np.linalg.norm(x_axis))
    x_axis = x_axis / max(x_norm, 1e-9)
    y_axis = np.cross(z_axis, x_axis)
    y_axis = y_axis / max(float(np.linalg.norm(y_axis)), 1e-9)

    tree = ET.parse(str(base_mjcf))
    root = tree.getroot()

    option = ensure_child(root, "option")
    if option.get("gravity") is None:
        option.set("gravity", "0 0 -9.81")

    asset = ensure_child(root, "asset")
    if asset.find("./material[@name='m_box']") is None:
        ET.SubElement(asset, "material", {"name": "m_box", "rgba": "0.9 0.2 0.2 1"})
    if asset.find("./material[@name='m_table']") is None:
        ET.SubElement(asset, "material", {"name": "m_table", "rgba": "0.8 0.8 0.8 1"})

    worldbody = ensure_child(root, "worldbody")
    if worldbody.find("./light[@name='main_light']") is None:
        ET.SubElement(worldbody, "light", {"name": "main_light", "pos": "0 0 1.5", "dir": "0 0 -1"})
    if worldbody.find("./camera[@name='cam_fixed']") is None:
        ET.SubElement(
            worldbody,
            "camera",
            {
                "name": "cam_fixed",
                "pos": f"{cam_pos[0]:.3f} {cam_pos[1]:.3f} {cam_pos[2]:.3f}",
                "xyaxes": (
                    f"{x_axis[0]:.6f} {x_axis[1]:.6f} {x_axis[2]:.6f} "
                    f"{y_axis[0]:.6f} {y_axis[1]:.6f} {y_axis[2]:.6f}"
                ),
            },
        )
    if worldbody.find("./geom[@name='floor']") is None:
        ET.SubElement(
            worldbody,
            "geom",
            {"name": "floor", "type": "plane", "size": "2 2 0.1", "rgba": "0.95 0.95 0.95 1"},
        )

    if worldbody.find("./geom[@name='table']") is None:
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": "table",
                "type": "box",
                "pos": f"{box_pos[0]:.3f} {box_pos[1]:.3f} {box_pos[2] - 0.03:.3f}",
                "size": "0.15 0.15 0.03",
                "material": "m_table",
                "contype": "1",
                "conaffinity": "1",
            },
        )

    if worldbody.find("./body[@name='box']") is None:
        b = ET.SubElement(
            worldbody,
            "body",
            {"name": "box", "pos": f"{box_pos[0]:.3f} {box_pos[1]:.3f} {box_pos[2]:.3f}"},
        )
        ET.SubElement(b, "freejoint", {"name": "box_free"})
        ET.SubElement(
            b,
            "geom",
            {
                "name": "box_geom",
                "type": "box",
                "size": "0.02 0.02 0.02",
                "material": "m_box",
                "mass": "0.05",
                "contype": "1",
                "conaffinity": "1",
            },
        )

    out = base_mjcf.with_name("piper_x_scene.xml")
    tree.write(str(out), encoding="utf-8", xml_declaration=True)

    verify = mujoco.MjModel.from_xml_path(str(out))
    print(str(out))
    print("nu", verify.nu, "njnt", verify.njnt, "nbody", verify.nbody)


if __name__ == "__main__":
    main()
