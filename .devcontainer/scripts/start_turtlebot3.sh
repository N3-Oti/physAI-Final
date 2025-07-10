#!/bin/bash

# TurtleBot3シミュレーションを起動するスクリプト（改良版）

echo "Setting up TurtleBot3 environment..."

# ROS2環境をセットアップ
source /opt/ros/humble/setup.bash

# TurtleBot3モデルを設定
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

# ワークスペースに移動
cd /home/vscode/ros2_ws

# パッケージをビルド
echo "Building ROS2 workspace..."
colcon build --packages-select physics_ai --cmake-args -DCMAKE_BUILD_TYPE=Release
if [ $? -eq 0 ]; then
    echo "Build successful!"
    source install/setup.bash
else
    echo "Build failed! Continuing with system packages only..."
fi

# Gazeboが利用可能かチェック
if ! command -v gazebo &> /dev/null; then
    echo "ERROR: Gazebo not found!"
    exit 1
fi

echo "Starting TurtleBot3 Gazebo simulation..."
echo "Please wait, Gazebo is loading..."

# Gazeboシミュレーションを起動（タイムアウト付き）
timeout 60 ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
GAZEBO_PID=$!

# Gazeboの起動を待つ
sleep 20

# Gazeboが起動したかチェック
if ps -p $GAZEBO_PID > /dev/null; then
    echo "✅ TurtleBot3 simulation started successfully!"
else
    echo "❌ TurtleBot3 simulation failed to start"
    exit 1
fi

# プロセスを生かし続ける
wait $GAZEBO_PID

echo "TurtleBot3 simulation started!"
echo "You can now use natural language commands to control the robot." 