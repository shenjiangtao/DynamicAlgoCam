#!/usr/bin/env bash
# batch_wrap_color_depth.sh
# 批量封装目录中文件名包含 "color" 与 "depth" 的 .h264 文件为 .mp4
# Usage: ./batch_wrap_color_depth.sh <dir> [max_jobs] [--paired-only]
# Example: ./batch_wrap_color_depth.sh ./captures 4

set -euo pipefail

if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <dir> [max_jobs] [--paired-only]"
  exit 1
fi

DIR="$1"
MAX_JOBS="${2:-4}"
PAIRED_ONLY=false
if [ "${3:-}" = "--paired-only" ]; then
  PAIRED_ONLY=true
fi

command -v ffmpeg >/dev/null 2>&1 || { echo "需要 ffmpeg，请先安装"; exit 1; }

# 输出目录
OUT_DIR="${DIR%/}/wrapped_mp4"
mkdir -p "$OUT_DIR"

# 临时工作目录
TMPDIR="$(mktemp -d)"
trap 'rm -rf "$TMPDIR"' EXIT

declare -A MAP_COLOR
declare -A MAP_DEPTH
declare -A MAP_ALL

# 生成匹配 key：去掉 color/depth 字样与多余的下划线/连字符，转小写
gen_key() {
  local name="$1"
  # 仅文件名，不含路径
  local base="$(basename "$name")"
  # 去掉扩展
  base="${base%.*}"
  # 转小写
  base="$(echo "$base" | tr '[:upper:]' '[:lower:]')"
  # 删除 color 或 depth（包括前后下划线或连字符）
  base="$(echo "$base" | sed -E 's/(_|-)?(color|depth)(_|-)?/_/g')"
  # 把连续下划线合并，去掉首尾下划线
  base="$(echo "$base" | sed -E 's/_+/_/g; s/^_//; s/_$//')"
  echo "$base"
}

# 遍历目录，收集 .h264 文件
while IFS= read -r -d '' file; do
  fname="$(basename "$file")"
  lower="$(echo "$fname" | tr '[:upper:]' '[:lower:]')"
  key="$(gen_key "$fname")"
  MAP_ALL["$file"]=1
  if echo "$lower" | grep -q "color"; then
    MAP_COLOR["$key"]="$file"
  fi
  if echo "$lower" | grep -q "depth"; then
    MAP_DEPTH["$key"]="$file"
  fi
done < <(find "$DIR" -maxdepth 1 -type f \( -iname "*.h264" -o -iname "*.264" \) -print0)

# 准备任务队列（pairs 和 单文件）
declare -a TASKS

for k in "${!MAP_COLOR[@]}"; do
  if [ -n "${MAP_DEPTH[$k]:-}" ]; then
    TASKS+=("PAIR|$k|${MAP_COLOR[$k]}|${MAP_DEPTH[$k]}")
  else
    TASKS+=("SINGLE_COLOR|$k|${MAP_COLOR[$k]}")
  fi
done

# 找出 depth-only（color 中未包含的）
for k in "${!MAP_DEPTH[@]}"; do
  if [ -z "${MAP_COLOR[$k]:-}" ]; then
    TASKS+=("SINGLE_DEPTH|$k|${MAP_DEPTH[$k]}")
  fi
done

# 如果只处理配对，过滤 TASKS
if [ "$PAIRED_ONLY" = true ]; then
  filtered=()
  for t in "${TASKS[@]}"; do
    if [[ "$t" == PAIR\|* ]]; then
      filtered+=("$t")
    fi
  done
  TASKS=("${filtered[@]}")
fi

# 封装函数：优先 copy，失败则转码
wrap_file() {
  local in="$1"
  local out="$2"
  local fps="$3"
  echo "[`date +%H:%M:%S`] 封装: $in -> $out"
  if ffmpeg -y -fflags +genpts -r "$fps" -i "$in" -c copy "$out" >/dev/null 2>&1; then
    echo "[`date +%H:%M:%S`] 无损封装成功: $out"
    return 0
  fi
  echo "[`date +%H:%M:%S`] 无损封装失败，尝试转码..."
  if ffmpeg -y -r "$fps" -i "$in" -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p "$out" >/dev/null 2>&1; then
    echo "[`date +%H:%M:%S`] 转码封装成功: $out"
    return 0
  fi
  echo "[`date +%H:%M:%S`] 封装失败: $in"
  return 1
}

# 并发控制
running_jobs=0
pids=()

start_task() {
  local task="$1"
  IFS='|' read -r typ key f1 f2 <<< "$task"
  case "$typ" in
    PAIR)
      # 输出文件名基于 key + suffix
      base="$(basename "$f1")"
      # try to preserve original naming: replace .h264 with .mp4 and keep original filename
      out1="${OUT_DIR}/$(basename "${f1%.*}").mp4"
      out2="${OUT_DIR}/$(basename "${f2%.*}").mp4"
      (
        wrap_file "$f1" "$out1" 30
        wrap_file "$f2" "$out2" 30
      ) &
      pid=$!
      ;;
    SINGLE_COLOR)
      out1="${OUT_DIR}/$(basename "${f1%.*}").mp4"
      (
        wrap_file "$f1" "$out1" 30
      ) &
      pid=$!
      ;;
    SINGLE_DEPTH)
      out1="${OUT_DIR}/$(basename "${f1%.*}").mp4"
      (
        wrap_file "$f1" "$out1" 30
      ) &
      pid=$!
      ;;
    *)
      echo "未知任务类型: $typ"
      return 1
      ;;
  esac
  pids+=("$pid")
  running_jobs=$((running_jobs+1))
}

# 启动任务，限制并发
echo "准备执行 ${#TASKS[@]} 个任务，最大并发 ${MAX_JOBS} ..."
idx=0
for t in "${TASKS[@]}"; do
  # 等待直到有空闲 slot
  while [ "$running_jobs" -ge "$MAX_JOBS" ]; do
    # 清理已结束的子进程计数
    new_pids=()
    running_jobs=0
    for pid in "${pids[@]:-}"; do
      if kill -0 "$pid" 2>/dev/null; then
        new_pids+=("$pid")
        running_jobs=$((running_jobs+1))
      fi
    done
    pids=("${new_pids[@]}")
    sleep 0.2
  done
  start_task "$t"
  idx=$((idx+1))
done

# 等待所有子进程完成
for pid in "${pids[@]:-}"; do
  wait "$pid" || true
done

echo "全部任务完成，输出目录: $OUT_DIR"
exit 0

