FROM airobotbook/ros2-desktop-ai-robot-book-humble:latest

# 新しいROS2 GPGキーを追加
RUN curl -sSL 'https://raw.githubusercontent.com/ros/rosdistro/master/ros.key' -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo 'deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main' > /etc/apt/sources.list.d/ros2.list

# 必要な追加パッケージ
USER root
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dotenv \
    && rm -rf /var/lib/apt/lists/*

# AI連携用Pythonパッケージ
RUN pip3 install --no-cache-dir google-generativeai

# ワークスペースを追加
COPY ./ros2_ws /home/ubuntu/ros2_ws
RUN chown -R ubuntu:ubuntu /home/ubuntu/ros2_ws

# supervisorログディレクトリの権限を修正（rootで実行する場合は不要だが念のため）
RUN mkdir -p /var/log/supervisor && chown -R root:root /var/log/supervisor && chmod -R 755 /var/log/supervisor

# ROS2ワークスペースの権限を修正（起動時のPermissionエラー防止）
RUN chown -R ubuntu:ubuntu /home/ubuntu/ros2_ws

# ROS2ワークスペースのビルド（イメージ作成時に一度だけ）
RUN source /opt/ros/humble/setup.bash && cd /home/ubuntu/ros2_ws && colcon build

# lib/physics_ai ディレクトリを作成し、binからシンボリックリンクを張る
RUN mkdir -p /home/ubuntu/ros2_ws/install/physics_ai/lib/physics_ai && \
    ln -sf /home/ubuntu/ros2_ws/install/physics_ai/bin/gemini_json_publisher /home/ubuntu/ros2_ws/install/physics_ai/lib/physics_ai/gemini_json_publisher && \
    ln -sf /home/ubuntu/ros2_ws/install/physics_ai/bin/json_command_interpreter /home/ubuntu/ros2_ws/install/physics_ai/lib/physics_ai/json_command_interpreter

# ENTRYPOINTを元に戻す
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/entrypoint.sh"]

# ubuntuユーザーに切り替え
# USER ubuntu
WORKDIR /home/ubuntu/ros2_ws 