#!/bin/bash
# MID360 PTP master 包装脚本 (配合 mid360-ptp4l.service 使用, 安装到 /usr/local/bin)
#
# 作用: 常驻探测雷达网口 (静态 IP 192.168.1.50, 候选 enx9c69d34b7003 /
# enx00e04c3605bd), 出现后以 PTP master 身份跑 ptp4l。MID360 靠 PTP 对时,
# 主机不跑 ptp4l 时时间戳基于雷达自由跑时钟, 长时间运行会失步 ->
# FAST-LIO "前半段正常后面突然飘" (2026-10-24 排查结论)。
# USB 网卡无硬件时间戳, ptp4l 自动走软件时间戳, LIO 够用。
#
# [2026-10-30 改] 不再"等 300s 退出再被 systemd 重启": 改成无限循环探测
# (1s 间隔)。雷达未插时服务安静 running (不再每 305s 一轮 Failed/重启刷屏,
# 之前 restart counter 能刷到 200+), 插上后 <=1s 拉起 ptp4l; 雷达拔掉后
# ptp4l 随网卡消失退出, Restart=always 拉起本脚本继续探测。
# CPU 占用可忽略 (sleep 为主, 每轮 ip+awk ~8ms)。
#
# 安装 (一次性, 在终端里跑):
#   sudo cp /home/xmy/final/src/mbf_test/systemd/mid360_ptp4l.sh /usr/local/bin/
#   sudo chmod +x /usr/local/bin/mid360_ptp4l.sh
#   sudo cp /home/xmy/final/src/mbf_test/systemd/mid360-ptp4l.service /etc/systemd/system/
#   sudo systemctl daemon-reload
#   sudo systemctl enable --now mid360-ptp4l
# 之后每次插雷达自动同步, 无需 sudo。查看状态: systemctl status mid360-ptp4l

while :; do
    IF=$(ip -4 -o addr show | awk '/192\.168\.1\.50/ {print $2; exit}')
    if [ -n "$IF" ]; then
        echo "mid360_ptp4l: 网口 $IF 就绪, 启动 PTP master"
        # 用 /usr/local/sbin 的 ptp4l 4.4 (2026-10-28 自编译); -S 软件时间戳:
        # USB 网卡无硬件时间戳, 不加 -S 直接报 "does not support requested
        # timestamping mode" 退出 -> 服务反复重启 (2026-10-27 实车: 重启计数
        # 1000+, 雷达时钟自由跑, 时间戳漂 10s+, costmap 查 TF 全失败不出图 /
        # 动态避障丢帧)
        # [2026-10-31 补丁] 4.4 在软件模式下仍会校验网卡上报的 so_timestamping,
        # r8152 USB 网卡只上报硬件位不含 SOFTWARE 位 -> 即使加 -S 也被误杀。
        # 已补丁 clock.c (TS_SOFTWARE 跳过该检查) 重编译安装到 /usr/local/sbin
        # (原版备份 ptp4l.bak-4.4-orig)。重装二进制必须带此补丁, 否则仍挂。
        exec /usr/local/sbin/ptp4l -A -S -i "$IF" -m
        # exec 失败或 ptp4l 退出 -> systemd 按 Restart=always 重启本脚本
    fi
    sleep 1
done
