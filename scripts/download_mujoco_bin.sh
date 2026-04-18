#!/usr/bin/env bash
set -euo pipefail

version="${1:-2.3.7}"
root_dir="$(cd "$(dirname "$0")/.." && pwd)"
dst_dir="${root_dir}/third_party"
archive="mujoco-${version}-linux-x86_64.tar.gz"
url="https://github.com/google-deepmind/mujoco/releases/download/${version}/${archive}"

mkdir -p "${dst_dir}"
cd "${dst_dir}"

if [[ -d "mujoco-${version}" ]]; then
  echo "已存在：${dst_dir}/mujoco-${version}"
  exit 0
fi

curl -L -o "${archive}" "${url}"
tar -xzf "${archive}"
rm -f "${archive}"
echo "完成：${dst_dir}/mujoco-${version}"
