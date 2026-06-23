#!/usr/bin/env bash
# detect_orbbec_usb.sh — 检测 Orbbec 与 RoboX 设备，输出 USB 带宽、设备路径、完整型号
set -uo pipefail

# 支持同时检测两个厂商 ID（按用户要求）
VID_ORBBEC_HEX="2bc5"
VID_ROBOX_DEC="3840"

echo "正在检测 Orbbec (VID=${VID_ORBBEC_HEX}) 与 RoboX (VID=${VID_ROBOX_DEC}) 设备..."

# 从 lsusb 中筛选包含任一 VID 的行（若 lsusb 输出中 VID 为十六进制字符串）
dev_all=$(lsusb | grep -E "${VID_ORBBEC_HEX}|${VID_ROBOX_DEC}" || true)

if [[ -z "$dev_all" ]]; then
    echo "未检测到 Orbbec 或 RoboX 设备"
    exit 0
fi

USB2_COUNT=0
USB3_COUNT=0

# 逐行读取 lsusb 原始整行，保留型号字段
while IFS= read -r line; do
    # 典型 lsusb 行: "Bus 002 Device 003: ID 2bc5:0400 Orbbec Astra"
    # 提取字段
    bus_str=$(echo "$line" | awk '{print $2}')
    dev_str=$(echo "$line" | awk '{print $4}')
    vidpid=$(echo "$line" | awk '{print $6}')   # 形如 2bc5:0400
    dev_name=$(echo "$line" | cut -d: -f3- | sed 's/^ //')

    # 清理 Bus/Dev 编号（去前导零）
    BUS_CLEAN=$(echo "$bus_str" | sed 's/^0*//')
    DEV_CLEAN=$(echo "$dev_str" | sed 's/^0*//')
    # 用两位格式化以匹配 lsusb -t 中的 Bus XX 表示
    BUS_2D=$(printf "%02d" "${BUS_CLEAN:-0}")

    USB_DEV_PATH="/dev/bus/usb/${bus_str}/${dev_str}"

    # 提取 VID（冒号前）
    VID_STR=$(echo "$vidpid" | cut -d: -f1 | tr '[:upper:]' '[:lower:]')

    # 识别厂商（优先按显式 VID 匹配）
    vendor="未知厂商"
    if [[ "$VID_STR" == "${VID_ORBBEC_HEX}" ]]; then
        vendor="Orbbec"
    elif echo "$line" | grep -q "${VID_ROBOX_DEC}"; then
        # 如果 lsusb 行中包含 3840 字符串（用户指定），标记为 RoboX
        vendor="RoboX"
    else
        # 额外尝试：若 VID_STR 是十六进制但等于 3840 的十六进制表示，则也识别
        # 将十六进制 VID_STR 转为十进制并比较
        if [[ "$VID_STR" =~ ^[0-9a-f]+$ ]]; then
            dec_vid=$((16#$VID_STR))
            if [[ "$dec_vid" -eq "$VID_ROBOX_DEC" ]]; then
                vendor="RoboX"
            fi
        fi
    fi

    # 从 lsusb -t 中尝试获取该设备的速率（匹配 Bus XX 和 Dev Y）
    speed=$(lsusb -t 2>/dev/null | grep -A6 "Bus $BUS_2D" | grep "Dev $DEV_CLEAN" | grep -oE '[0-9]+M' || true)
    case "$speed" in
        480M)
            label="USB2.0"
            ((USB2_COUNT++))
            ;;
        5000M|10000M)
            label="USB3.x"
            ((USB3_COUNT++))
            ;;
        *)
            label="未知速率"
            ;;
    esac

    echo "========================================"
    echo "厂商识别: $vendor"
    echo "设备型号: $dev_name"
    echo "VID:PID: $vidpid"
    echo "总线信息: Bus ${BUS_CLEAN:-0} Dev ${DEV_CLEAN:-0} (${label})"
    echo "USB 节点路径: $USB_DEV_PATH"
done <<< "$dev_all"

echo "========================================"
echo
echo "汇总统计: USB2.0设备=$USB2_COUNT 台, USB3.x设备=$USB3_COUNT 台"

