from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco


def add_motor_actuators(xml_path: Path, joint_names: list[str]) -> None:
    tree = ET.parse(str(xml_path))
    root = tree.getroot()

    actuator = root.find("actuator")
    if actuator is None:
        actuator = ET.SubElement(root, "actuator")

    existing = {elem.get("joint") for elem in actuator.findall("motor")}
    for j in joint_names:
        if j in existing:
            continue
        ET.SubElement(
            actuator,
            "motor",
            {
                "name": f"m_{j}",
                "joint": j,
                "gear": "1",
            },
        )

    tree.write(str(xml_path), encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--arm",
        default="piper_x",
        choices=["piper_x", "piper", "piper_l", "piper_h", "nero"],
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    urdf_dir = repo_root / "models" / "agx_arm_ros_urdf" / args.arm
    urdf_path = urdf_dir / f"{args.arm}_description.urdf"
    if not urdf_path.exists():
        raise FileNotFoundError(
            f"找不到 URDF：{urdf_path}\n"
            "提示：先运行 python scripts/prepare_urdf_for_mujoco.py --arm piper_x"
        )

    model = mujoco.MjModel.from_xml_path(str(urdf_path))
    mjcf_path = urdf_dir / f"{args.arm}_mujoco.xml"
    mujoco.mj_saveLastXML(str(mjcf_path), model)

    joint_names = [
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in range(model.njnt)
    ]
    joint_names = [j for j in joint_names if j]
    add_motor_actuators(mjcf_path, joint_names)

    verify = mujoco.MjModel.from_xml_path(str(mjcf_path))
    print(str(mjcf_path))
    print("nu", verify.nu, "njnt", verify.njnt)


if __name__ == "__main__":
    main()
