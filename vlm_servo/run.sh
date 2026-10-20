#!/usr/bin/env bash
# 用 FoundationPose 这个 conda 环境运行阶段一。
# 如果环境名不同：VLM_ENV=你的环境名 ./run.sh
set -e
cd "$(dirname "$0")"

ENV_NAME="${VLM_ENV:-FoundationPose}"

# 直接调用该环境的 python，避免 `conda run` 干扰交互式 input()/cv2 窗口
PYTHON=""
for prefix in "$HOME/anaconda3" "$HOME/miniconda3" /opt/anaconda3 /opt/miniconda3; do
  cand="$prefix/envs/$ENV_NAME/bin/python"
  if [ -x "$cand" ]; then PYTHON="$cand"; break; fi
done

if [ -z "$PYTHON" ]; then
  if [ -n "$CONDA_PREFIX" ] && [ -x "$CONDA_PREFIX/bin/python" ]; then
    PYTHON="$CONDA_PREFIX/bin/python"
  else
    echo "[!] 找不到环境 '$ENV_NAME' 的 python。"
    echo "    请用 'conda env list' 确认环境名，或用 VLM_ENV=名字 ./run.sh"
    exit 1
  fi
fi

echo "[run] 使用 $PYTHON"
exec "$PYTHON" main.py "$@"
