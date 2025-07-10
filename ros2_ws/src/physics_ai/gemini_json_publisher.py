#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # JSON文字列をパブリッシュするために使用
import google.generativeai as genai
import os
import sys
import json # JSONのバリデーション用
from dotenv import load_dotenv
import re
import time

# 追加: AI設定の読み込み関数
def load_ai_config(config_path="../ai_config.json"):
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            config = json.load(f)
        return config.get("model", "gemini-2.5-flash"), config.get("system_prompt", "")
    except Exception as e:
        print(f"[ERROR] ai_config.jsonの読み込みに失敗: {e}")
        return "gemini-2.5-flash", ""

# コードブロックからJSONだけを抽出する関数
def extract_json_from_codeblock(text):
    match = re.search(r"```json\\s*(.*?)\\s*```", text, re.DOTALL)
    if match:
        return match.group(1)
    # それ以外はそのまま返す
    return text

# ai_config.jsonの読み込み部分を拡張
with open('ai_config.json', 'r', encoding='utf-8') as f:
    ai_config = json.load(f)
    model_name = ai_config.get('model', '')
    system_prompt = ai_config.get('system_prompt', '')
    preprocess_model = ai_config.get('preprocess_model', '')
    preprocess_prompt = ai_config.get('preprocess_prompt', '')


class GeminiJsonPublisher(Node):
    def __init__(self):
        super().__init__('gemini_json_publisher')
        self.publisher_ = self.create_publisher(String, '/robot_commands_json', 10)
        self.get_logger().info('Gemini JSON Publisher Node has been started.')
        
        load_dotenv() # .envファイルを読み込む  
        # Gemini APIキーの取得
        gemini_api_key = os.getenv("GEMINI_API_KEY")
        if not gemini_api_key:
            self.get_logger().error("GEMINI_API_KEY environment variable not set. Please create a .env file or set the environment variable.")
            raise ValueError("GEMINI_API_KEY is not set.")
        
        # ai_config.jsonからモデル名とシステムプロンプトを取得
        self.gemini_model_name, self.system_prompt = load_ai_config()
        # system_promptをTurtleBot3/turtlesim命令に限定
        self.system_prompt = (
            "あなたはロボット制御AIです。"
            "与えられた『具体的な動作説明（自然言語）』を、必ずTurtleBot3やturtlesimで処理できるJSON命令列に変換してください。"
            "使える命令は以下のみです：\n"
            "- {\"action\": \"move\", \"distance\": 距離（m, float）}\n"
            "- {\"action\": \"turn\", \"angle\": 角度（度, float。正:右回転、負:左回転）}\n"
            "- {\"action\": \"stop\"}\n"
            "説明や解説、コードブロックは不要です。JSON配列のみを出力してください。\n"
            "【例】\n"
            "入力文：右に15度回転して1メートル進み、左に30度回転して1メートル進む動作を4回繰り返す\n"
            "出力：\n[\n  {\"action\": \"turn\", \"angle\": 15},\n  {\"action\": \"move\", \"distance\": 1.0},\n  {\"action\": \"turn\", \"angle\": -30},\n  {\"action\": \"move\", \"distance\": 1.0},\n  {\"action\": \"turn\", \"angle\": 15},\n  {\"action\": \"move\", \"distance\": 1.0},\n  {\"action\": \"turn\", \"angle\": -30},\n  {\"action\": \"move\", \"distance\": 1.0}\n]"
        )
        # preprocess_model, preprocess_promptもセット
        with open('ai_config.json', 'r', encoding='utf-8') as f:
            ai_config = json.load(f)
            self.preprocess_model = ai_config.get('preprocess_model', '')
            self.preprocess_prompt = ai_config.get('preprocess_prompt', '')
        self.get_logger().info(f"Using Gemini model: {self.gemini_model_name}")
        genai.configure(api_key=gemini_api_key)

        # Gemini APIのJSONモードを有効化
        self.generation_config = {"response_mime_type": "application/json"}

    def preprocess_input(self, user_input):
        if not self.preprocess_model:
            return user_input
        prompt = self.preprocess_prompt.replace('{input}', user_input)
        response = self.get_gemini_response(prompt)
        return response

    # Gemini APIでpromptを入力して返信を受け取る
    def get_gemini_response(self, prompt):
        model = genai.GenerativeModel(self.gemini_model_name, generation_config=self.generation_config)
        try:
            response = model.generate_content(prompt)
            if response.parts:
                return response.text.strip()
            else:
                self.get_logger().warn("Gemini API did not return any text parts.")
                return ""
        except Exception as e:
            self.get_logger().error(f"Error calling Gemini API: {e}")
            return ""

    def publish_json_command(self, json_string):
        msg = String()
        msg.data = json_string
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published JSON command: "{json_string}"')

    def send_command_to_gemini(self, user_input):
        if not self.system_prompt:
            self.get_logger().error("System prompt is not loaded. Cannot send command to Gemini.")
            return

        # 前処理を挟む
        processed_input = self.preprocess_input(user_input)
        time.sleep(6)  # Gemini APIのRPM制限回避のため6秒待機
        full_prompt = self.system_prompt + "\nUser: " + processed_input + "\nAssistant:\n"
        self.get_logger().info(f"Sending prompt to Gemini:\n{full_prompt}")
        
        raw_response = self.get_gemini_response(full_prompt)
        
        if raw_response:
            self.get_logger().info(f"Gemini raw response:\n{raw_response}\n")
            try:
                # コードブロックがあれば除去してからパース
                json_str = extract_json_from_codeblock(raw_response)
                parsed_json = json.loads(json_str)
                # パースできたJSONを整形して再度文字列化（必要であれば）
                formatted_json_string = json.dumps(parsed_json, indent=2) 
                self.publish_json_command(formatted_json_string)
            except json.JSONDecodeError:
                self.get_logger().error(f"Gemini did not return valid JSON: {raw_response}")
            except Exception as e:
                self.get_logger().error(f"Error processing Gemini response: {e}")
        else:
            self.get_logger().warn("No response from Gemini.")

def main(args=None):
    rclpy.init(args=args)
    node = GeminiJsonPublisher()

    try:
        if len(sys.argv) > 1:
            user_prompt = " ".join(sys.argv[1:])
            node.send_command_to_gemini(user_prompt)
            rclpy.spin_once(node, timeout_sec=1.0)
        else:
            print("Usage: ros2 run YOUR_PACKAGE_NAME gemini_json_publisher 'Your command'")
            print("Or run in interactive mode by providing no arguments:")
            while rclpy.ok():
                user_input = input("Enter your command for the robot: ")
                if user_input.lower() == 'exit':
                    break
                node.send_command_to_gemini(user_input)
                rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()