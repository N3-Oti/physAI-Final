#!/bin/bash

# 全システムを起動するメインスクリプト（改良版）

echo "Starting PhysAI Robot Control System..."

# ディレクトリを移動
cd /home/vscode/scripts

# VNC環境を起動
./start_vnc.sh &
VNC_PID=$!

echo "Waiting for VNC to start..."
sleep 10

# ROS2環境をセットアップ
source /opt/ros/humble/setup.bash

# TurtleBot3シミュレーションを起動
./start_turtlebot3.sh &
TURTLEBOT_PID=$!

echo "Waiting for TurtleBot3 simulation to start..."
sleep 30

# JSONコマンドインタープリターを起動
cd /home/vscode/ros2_ws
source /opt/ros/humble/setup.bash
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
    ros2 run physics_ai json_command_interpreter &
    JSON_PID=$!
    echo "✅ JSON Command Interpreter started"
else
    echo "⚠️ Package not built, skipping JSON Command Interpreter"
fi

echo "System startup complete!"
echo "======================================"
echo "🚀 PhysAI Robot Control System Ready!"
echo "======================================"
echo "🌐 Web Access: http://localhost:6080/vnc.html"
echo "🔐 VNC Password: physai123"
echo "🤖 Robot Control: Ready for natural language commands"
echo "======================================"

# プロセスを生かし続ける
wait $VNC_PID $TURTLEBOT_PID 