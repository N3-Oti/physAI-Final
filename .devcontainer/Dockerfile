# OSRF公式のROS 2 Humble + Desktop環境をベースイメージとして使用
FROM osrf/ros:humble-desktop

# 必要なパッケージとLXDEデスクトップ環境を追加インストール
RUN apt-get update && apt-get install -y \
    build-essential \
    python3-colcon-common-extensions \
    python3-pip \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-simulations \
    bash \
    x11vnc \
    xvfb \
    lxde \
    xterm \
    x11-utils \
    novnc \
    websockify \
    && rm -rf /var/lib/apt/lists/*

# 開発に必要なPythonパッケージをインストール
RUN pip3 install --no-cache-dir python-dotenv google-generativeai

# VSCodeで作成したファイルの所有権問題を回避するため、一般ユーザーを作成
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# noVNC環境設定
ENV DISPLAY=:1
ENV VNC_PORT=5901
ENV NOVNC_PORT=6080
ENV RESOLUTION=1400x900

# VNC起動スクリプトを作成
COPY .devcontainer/scripts/ /home/vscode/scripts/
RUN chmod +x /home/vscode/scripts/*.sh \
    && chown -R $USERNAME:$USERNAME /home/vscode/scripts/

# 作成したユーザーに切り替え
USER $USERNAME

# ROS 2のセットアップスクリプトを自動で読み込むように設定
RUN echo "source /opt/ros/humble/setup.bash" >> /home/vscode/.bashrc

# TurtleBot3環境変数設定
RUN echo "export TURTLEBOT3_MODEL=waffle" >> /home/vscode/.bashrc \
    && echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models" >> /home/vscode/.bashrc

# コンテナ内ワークスペースを作成
WORKDIR /home/vscode/ros2_ws

# VNCサーバー起動用のポート公開
EXPOSE 5901 6080