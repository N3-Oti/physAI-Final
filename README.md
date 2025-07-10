# PhysicalAI ROS2 + Gemini + turtlesim

## 概要

このプロジェクトは、Google Gemini APIとROS2を連携し、自然言語でturtlesimを制御できるシミュレーション環境をDocker上で提供します。

- Gemini APIによる自然言語→JSONコマンド変換
- ROS2ノードによるコマンド解釈・ロボット制御
- noVNC経由でGUIデスクトップにアクセス
- turtlesimのシミュレーション

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

## AIによるロボットコマンド生成の流れ（2024/07更新）

1. **ユーザーが自然言語で曖昧な指示を入力**
   - 例：「ジグザグに動け」「ぐるぐる回って」
2. **前処理AIが具体的な動作説明に変換**
   - 例：「右に15度回転して1メートル進み、左に30度回転して1メートル進む動作を4回繰り返す」
   - ※使える表現は「角度（度）」「距離（メートル）」「回数」のみ。速度表現は禁止。
3. **本体AIがJSON命令列に変換**
   - 例：
```
[
  {"action": "turn", "angle": 15},
  {"action": "move", "distance": 1.0},
  {"action": "turn", "angle": -30},
  {"action": "move", "distance": 1.0},
  {"action": "turn", "angle": 15},
  {"action": "move", "distance": 1.0},
  {"action": "turn", "angle": -30},
  {"action": "move", "distance": 1.0}
]
```
   - ※使える命令は `move`（距離m）, `turn`（角度deg）, `stop` のみ。`action`キーで判定。
1. **json_command_interpreter.pyがJSON命令列を解釈し、turtlesim等にコマンドを送信**
   - `move`, `turn`, `stop` のみ対応。`action`キーで判定。
   - 速度指定やその他のパラメータは無視される。

## ai_config.json 設計例

```
{
  "model": "gemini-1.5-pro",
  "system_prompt": "あなたはロボット制御AIです。与えられた『具体的な動作説明（自然言語）』を、必ずTurtleBot3やturtlesimで処理できるJSON命令列に変換してください。使える命令は以下のみです：\n- {\"action\": \"move\", \"distance\": 距離（m, float）}\n- {\"action\": \"turn\", \"angle\": 角度（度, float。正:右回転、負:左回転）}\n- {\"action\": \"stop\"}\n説明や解説、コードブロックは不要です。JSON配列のみを出力してください。\n【例】\n入力文：右に15度回転して1メートル進み、左に30度回転して1メートル進む動作を4回繰り返す\n出力：\n[\n  {\"action\": \"turn\", \"angle\": 15},\n  {\"action\": \"move\", \"distance\": 1.0},\n  {\"action\": \"turn\", \"angle\": -30},\n  {\"action\": \"move\", \"distance\": 1.0},\n  {\"action\": \"turn\", \"angle\": 15},\n  {\"action\": \"move\", \"distance\": 1.0},\n  {\"action\": \"turn\", \"angle\": -30},\n  {\"action\": \"move\", \"distance\": 1.0}\n]\n入力文：その場で360度回転する動作を2回繰り返す\n出力：\n[\n  {\"action\": \"turn\", \"angle\": 360},\n  {\"action\": \"turn\", \"angle\": 360}\n]\n入力文：2メートル直進する\n出力：\n[\n  {\"action\": \"move\", \"distance\": 2.0}\n]\n",
  "preprocess_model": "gemini-1.5-pro",
  "preprocess_prompt": "あなたはロボットの動作プランナーです。ユーザーの曖昧な指示を、必ず『具体的な動作手順（自然言語）』に変換してください。使ってよい表現は『角度（度）』『距離（メートル）』『回数』のみです。『速く』『ゆっくり』などの速度表現や曖昧な表現は禁止です。『右に◯度回転して◯メートル進む』『◯回繰り返す』など、具体的な命令文にしてください。説明や解説は不要です。動作手順のみを簡潔に出力してください。\n\n【例】\n入力文「ジグザグに動け」\n出力：右に15度回転して1メートル進み、左に30度回転して1メートル進む動作を4回繰り返す\n\n入力文「ぐるぐる回って」\n出力：その場で360度回転する動作を2回繰り返す\n\n入力文「シュッと進んで」\n出力：2メートル直進する\n\n入力文「{input}」"
}
```

## 注意事項・運用ルール
- AI出力のJSON仕様（actionキー、move/turn/stopのみ）とインタプリタ実装は必ず一致させること。


---

## 参考
- [Gemini API 公式ドキュメント](https://ai.google.dev/gemini-api/docs?hl=ja)
- [ROS2公式ドキュメント](https://docs.ros.org/en/)

---
