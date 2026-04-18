#!/usr/bin/env bash
set -euo pipefail

: "${REPO_URL:?请先设置 REPO_URL，例如：export REPO_URL=https://github.com/xxx/yyy.git}"

mkdir -p repos
cd repos

name="$(basename "${REPO_URL}")"
name="${name%.git}"

if [[ -d "${name}" ]]; then
  echo "已存在：repos/${name}"
  exit 0
fi

git clone --depth=1 "${REPO_URL}" "${name}"
echo "完成：repos/${name}"
