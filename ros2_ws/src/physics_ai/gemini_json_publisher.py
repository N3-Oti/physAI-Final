#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # JSON文字列をパブリッシュするために使用
import google.generativeai as genai
import os
import sys
import json # JSONのバリデーション用
from dotenv import load_dotenv

# 追加: AI設定の読み込み関数
def load_ai_config(config_path="../ai_config.json"):
    try:
        with open(config_path, "r", encoding="utf-8") as f:
            config = json.load(f)
        return config.get("model", "gemini-2.5-flash"), config.get("system_prompt", "")
    except Exception as e:
        print(f"[ERROR] ai_config.jsonの読み込みに失敗: {e}")
        return "gemini-2.5-flash", ""

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
        self.gemini_model_name, self.system_prompt = load_ai_config(os.path.join(os.path.dirname(__file__), '../../ai_config.json'))
        self.get_logger().info(f"Using Gemini model: {self.gemini_model_name}")
        genai.configure(api_key=gemini_api_key)

    # Gemini APIでpromptを入力して返信を受け取る
    def get_gemini_response(self, prompt):
        model = genai.GenerativeModel(self.gemini_model_name)
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

        full_prompt = self.system_prompt + "\nUser: " + user_input + "\nAssistant:\n"
        self.get_logger().info(f"Sending prompt to Gemini:\n{full_prompt}")
        
        raw_response = self.get_gemini_response(full_prompt)
        
        if raw_response:
            self.get_logger().info(f"Gemini raw response:\n{raw_response}\n")
            try:
                # JSONとしてパースできるか検証
                parsed_json = json.loads(raw_response)
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