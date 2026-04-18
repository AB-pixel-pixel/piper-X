from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import argparse
import re
import shutil


@dataclass(frozen=True)
class PackageMapping:
    package: str
    root: Path


def rewrite_package_urls(urdf_text: str, mappings: list[PackageMapping]) -> str:
    out = urdf_text
    for m in mappings:
        prefix = f"package://{m.package}/"
        out = out.replace(prefix, str(m.root.as_posix()) + "/")
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--arm",
        default="piper_x",
        choices=["piper_x", "piper", "piper_l", "piper_h", "nero"],
    )
    args = parser.parse_args()

    repo_root = Path(__file__).resolve().parents[1]
    agx_desc_root = (
        repo_root
        / "repos"
        / "agx_arm_ros"
        / "src"
        / "agx_arm_description"
    )

    src_urdf = (
        agx_desc_root
        / "agx_arm_urdf"
        / args.arm
        / "urdf"
        / f"{args.arm}_description.urdf"
    )
    if not src_urdf.exists():
        raise FileNotFoundError(
            f"找不到 URDF：{src_urdf}\n"
            "提示：先在 repos/agx_arm_ros 下执行 git submodule update --init --recursive"
        )

    dst_dir = repo_root / "models" / "agx_arm_ros_urdf" / args.arm
    dst_dir.mkdir(parents=True, exist_ok=True)
    dst_urdf = dst_dir / f"{args.arm}_description.urdf"

    text = src_urdf.read_text(encoding="utf-8")
    text = rewrite_package_urls(
        text,
        mappings=[
            PackageMapping(
                package="agx_arm_description",
                root=agx_desc_root,
            )
        ],
    )

    mesh_paths: list[Path] = []
    for m in re.finditer(r'<mesh\s+filename="([^"]+)"\s*/?>', text):
        mesh_paths.append(Path(m.group(1)))

    seen_basename_to_src: dict[str, Path] = {}
    for src in mesh_paths:
        if not src.is_absolute():
            continue
        base = src.name
        if base in seen_basename_to_src and seen_basename_to_src[base] != src:
            raise RuntimeError(
                f"检测到 mesh 文件同名冲突：{base}\n"
                f"- {seen_basename_to_src[base]}\n"
                f"- {src}\n"
                "请改为复制到不同目录或手动重命名。"
            )
        seen_basename_to_src[base] = src

    for base, src in seen_basename_to_src.items():
        dst = dst_dir / base
        if not dst.exists():
            if not src.exists():
                raise FileNotFoundError(f"mesh 文件不存在：{src}")
            shutil.copyfile(src, dst)

    for base, src in seen_basename_to_src.items():
        text = text.replace(str(src.as_posix()), base)

    dst_urdf.write_text(text, encoding="utf-8")
    print(str(dst_urdf))


if __name__ == "__main__":
    main()
