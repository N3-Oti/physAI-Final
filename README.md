# PhysAI - Natural Language Robot Control

ROS2とGemini AIを統合した自然言語ロボット制御システム

## 🚀 使用方法

### 1. 環境設定
```bash
# Gemini API キーを設定
cp .env_example .env
# .envファイルにGEMINI_API_KEYを設定してください
```

### 2. システム起動
```bash
docker-compose up --build
```

起動には数分かかります。以下のメッセージが表示されるまでお待ちください：
```
🚀 PhysAI Robot Control System Ready!
🌐 Web Access: http://localhost:6080/vnc.html
🤖 Robot Control: Ready for natural language commands
```

### 3. ブラウザアクセス
http://localhost:6080/vnc.html にアクセス
- **VNCパスワード不要**
- Gazebo シミュレーターでTurtleBot3が表示されます

### 4. 自然言語でロボット制御
新しいターミナルでコンテナに接続：
```bash
docker exec -it ros2_gazebo_container bash
cd /home/vscode/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# 自然言語コマンドの例
ros2 run physics_ai gemini_json_publisher "前に進んで"
ros2 run physics_ai gemini_json_publisher "左に回転して3秒間"
ros2 run physics_ai gemini_json_publisher "右に曲がって5秒間"
ros2 run physics_ai gemini_json_publisher "停止"
ros2 run physics_ai gemini_json_publisher "後ろに下がって"
```

## 🛠️ システム構成

- **ROS2 Humble**: ロボット制御フレームワーク
- **TurtleBot3**: シミュレーションロボット  
- **Gazebo**: 物理シミュレーション環境
- **Gemini AI**: 自然言語処理
- **noVNC**: ブラウザベースのGUIアクセス
- **Docker**: コンテナ化環境

## 🏗️ アーキテクチャ

```
自然言語入力 → Gemini AI → JSON変換 → ROS2 Topic → TurtleBot3制御
```

1. **gemini_json_publisher**: 自然言語をGemini AIでJSONコマンドに変換
2. **json_command_interpreter**: JSONコマンドをROS2のcmd_velトピックに変換
3. **TurtleBot3**: Gazebo内でロボットが実際に動作

## 📁 プロジェクト構造

```
physAI-Final/
├── .devcontainer/
│   ├── Dockerfile              # Docker環境設定
│   └── scripts/                # 起動スクリプト
│       ├── start_vnc.sh        # VNC環境起動
│       ├── start_turtlebot3.sh # TurtleBot3起動
│       └── start_system.sh     # 全システム起動
├── ros2_ws/                    # ROS2ワークスペース
│   ├── src/
│   │   ├── physics_ai/             # ROS2パッケージ
│   │   └── resource/               # パッケージリソース
│   ├── package.xml
│   └── setup.py
├── docker-compose.yaml         # Docker Compose設定
├── prompt.txt                  # Gemini AI用プロンプト
├── .env_example
└── README.md
```

## 🐛 トラブルシューティング

### Gazeboが起動しない場合
```bash
# コンテナ内で手動起動
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### 自然言語コマンドが反応しない場合
```bash
# JSONコマンドインタープリターの状態確認
ros2 topic list | grep robot_commands_json
ros2 topic echo /robot_commands_json
```

### VNCに接続できない場合
- ポート6080が他のプロセスで使用されていないか確認
- ファイアウォール設定を確認

## 🔧 開発者向け情報

### パッケージのビルド
```bash
cd /home/vscode/ros2_ws
colcon build --packages-select physics_ai
source install/setup.bash
```

### ログ確認
```bash
ros2 node list
ros2 topic list
ros2 log show gemini_json_publisher
ros2 log show json_command_interpreter
```