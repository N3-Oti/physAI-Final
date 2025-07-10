#!/bin/bash

# VNC環境を起動するスクリプト（LXDEデスクトップ対応）

echo "Starting VNC environment..."

# Xvfbを起動（仮想Xサーバー）
export DISPLAY=:1
Xvfb :1 -screen 0 ${RESOLUTION:-1400x900}x16 &
XVFB_PID=$!

# 少し待ってからLXDEデスクトップ環境を起動
sleep 2
startlxde &
LXDE_PID=$!

# VNCサーバーをパスワードなしで起動
x11vnc -display :1 -forever -nopw -create -rfbport ${VNC_PORT:-5901} &
VNC_PID=$!

# noVNCを起動
websockify --web /usr/share/novnc/ ${NOVNC_PORT:-6080} localhost:${VNC_PORT:-5901} &
NOVNC_PID=$!

echo "VNC Server started on port ${VNC_PORT:-5901} (no password)"
echo "noVNC Web interface available at http://localhost:${NOVNC_PORT:-6080}/vnc.html"

# プロセスの終了を待つ
wait $XVFB_PID $LXDE_PID $VNC_PID $NOVNC_PID 