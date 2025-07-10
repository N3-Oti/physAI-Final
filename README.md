# PhysicalAI ROS2 + Gemini + TurtleBot3 Simulation

## 概要

このプロジェクトは、Google Gemini APIとROS2を連携し、自然言語でTurtleBot3やturtlesimを制御できるシミュレーション環境をDocker上で提供します。

- Gemini APIによる自然言語→JSONコマンド変換
- ROS2ノードによるコマンド解釈・ロボット制御
- noVNC経由でGUIデスクトップにアクセス
- TurtleBot3やturtlesimのシミュレーション

---

## セットアップ手順

### 1. 必要ファイルの準備
- `.env` : Gemini APIキーなどを記載（例: `GEMINI_API_KEY=...`）
- `ai_config.json` : モデル名・システムプロンプトをJSONで管理

### 2. Dockerイメージのビルド
```bash
# プロジェクトルートで
docker-compose build --no-cache
```

### 3. コンテナの起動
```bash
docker-compose up
```

### 4. noVNCでデスクトップにアクセス
- ブラウザで `http://localhost:6080` にアクセス
- LXTerminal等でターミナルを開く

### 5. ROS2環境のセットアップ
- 新しいターミナルごとに必ず:
```bash
source ~/ros2_ws/install/setup.bash
```

---

## ノードの起動例

### 1. turtlesimノード
```bash
ros2 run turtlesim turtlesim_node
```

### 2. コマンド解釈ノード
```bash
ros2 run physics_ai json_command_interpreter
```

### 3. Gemini連携ノード（自然言語→JSON）
```bash
ros2 run physics_ai gemini_json_publisher
```
- インタラクティブモードで「Enter your command for the robot:」と表示されます。
- 例: `ジグザグに動かして` などと入力

---

## Gemini API連携の仕組み

- `ai_config.json` でモデル名・システムプロンプトを一元管理
- Gemini APIはJSONモードで呼び出し、返答にコードブロックが混ざっても自動で除去
- 生成されたJSONコマンドは `/robot_commands_json` トピックにパブリッシュ
- `json_command_interpreter` ノードがJSON配列を順次解釈し、Twistメッセージでロボットを制御

---

## トラブルシューティング

- **No executable found**
  - `colcon build --symlink-install` 後、`source ~/ros2_ws/install/setup.bash` を必ず実行
  - `lib/physics_ai/` ディレクトリが無い場合は自動でシンボリックリンクが作成される（Dockerfileで対応済み）
- **Permission denied**
  - `ros2_ws`ディレクトリの権限を`ubuntu:ubuntu`に修正（Dockerfileで自動化済み）
- **Geminiの返答がパースできない**
  - JSONモード＋コードブロック除去で堅牢に対応済み
- **ノード間通信ができない**
  - トピック名・型・source忘れ・ノード起動状態を最優先で確認

---

## 知見・ルールまとめ

- AI設定やプロンプトはJSONで一元管理し、運用・拡張性を高める
- Gemini APIはJSONモード＋前処理で堅牢にパース
- ノード追加・修正時は必ず再ビルド・source
- ノードごとに新しいターミナルを開き、毎回sourceを忘れずに
- Dockerfileで権限修正・シンボリックリンク作成を自動化し、運用トラブルを未然に防ぐ
- セキュリティ上、.envやAPIキーは.gitignore/.dockerignoreで管理

---

## 参考
- [Gemini API 公式ドキュメント](https://ai.google.dev/gemini-api/docs?hl=ja)
- [ROS2公式ドキュメント](https://docs.ros.org/en/)

---

何か問題があれば、エラーメッセージや状況を添えてご相談ください！