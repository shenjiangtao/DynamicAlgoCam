#!/usr/bin/env bash
# detect_orbbec_usb.sh — 检测 Orbbec 设备，输出USB带宽、设备路径、完整型号
set -uo pipefail

ORBBEC_VID="2bc5"
echo "正在检测 Orbbec 设备..."

dev_all=$(lsusb | grep "$ORBBEC_VID" || true)

if [[ -z "$dev_all" ]]; then
    echo "未检测到 Orbbec 设备"
    exit 0
fi

USB2_COUNT=0
USB3_COUNT=0

# 逐行读取 lsusb 原始整行，保留型号字段
while read -r line; do
    # 拆分字段：Bus 00X Device 00X: ID VID:PID 设备名称...
    word1=$(echo "$line" | awk '{print $1}')
    bus_str=$(echo "$line" | awk '{print $2}')
    word3=$(echo "$line" | awk '{print $3}')
    dev_str=$(echo "$line" | awk '{print $4}')
    # 截取冒号之后所有内容 = 完整设备型号
    dev_name=$(echo "$line" | cut -d: -f3- | sed 's/^ //')

    BUS_CLEAN=$(echo "$bus_str" | sed 's/^0*//')
    DEV_CLEAN=$(echo "$dev_str" | sed 's/^0*//')
    BUS_2D=$(echo "$bus_str" | sed -E 's/^0*([0-9]{2})$/\1/')
    USB_DEV_PATH="/dev/bus/usb/${bus_str}/${dev_str}"

    speed=$(lsusb -t | grep "Bus $BUS_2D\." | grep -oE '[0-9]+M' || true)
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
    echo "设备型号: $dev_name"
    echo "总线信息: Bus $BUS_CLEAN Dev $DEV_CLEAN ($label)"
    echo "USB节点路径: $USB_DEV_PATH"
done <<< "$dev_all"

echo "========================================"
echo
echo "汇总统计: USB2.0设备=$USB2_COUNT 台, USB3.x设备=$USB3_COUNT 台"

