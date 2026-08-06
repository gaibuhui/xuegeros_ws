#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
agent_dir="$repo_root/src/micro-ROS-Agent"
patch_file="$repo_root/patches/micro-ros-agent-darwin-logger.patch"

if ! git -C "$agent_dir" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "micro-ROS-Agent submodule is not initialized: git submodule update --init --recursive" >&2
  exit 1
fi

if git -C "$agent_dir" apply --reverse --check "$patch_file"; then
  echo "micro-ROS-Agent Darwin logger patch is already applied."
elif git -C "$agent_dir" apply --check "$patch_file"; then
  git -C "$agent_dir" apply "$patch_file"
  echo "Applied micro-ROS-Agent Darwin logger patch."
else
  echo "micro-ROS-Agent does not match the patch base; update the patch for the pinned revision." >&2
  exit 1
fi
