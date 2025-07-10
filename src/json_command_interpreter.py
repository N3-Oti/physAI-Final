import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import Twist
from std_msgs.msg import String # JSON文字列を受信するために使用
import json
import threading
import time

class JsonCommandInterpreter(Node):
    def __init__(self):
        super().__init__('json_command_interpreter')
        self.subscription = self.create_subscription(
            String,
            '/robot_commands_json', # JSONコマンドを受信するトピック
            self.json_command_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = None
        self.current_cmd_vel = Twist()
        self.stop_requested = False
        self.command_thread = None

        self.get_logger().info('JSON Command Interpreter Node has been started.')

    def json_command_callback(self, msg):
        self.get_logger().info(f'Received JSON: "{msg.data}"')
        try:
            command_data = json.loads(msg.data)
            command_type = command_data.get('command_type')
            parameters = command_data.get('parameters', {})

            if command_type == 'move':
                linear_x = float(parameters.get('linear_x', 0.0))
                angular_z = float(parameters.get('angular_z', 0.0))
                duration_s = float(parameters.get('duration_s', 5.0)) # デフォルト5秒

                self.get_logger().info(f'Executing move command: linear_x={linear_x}, angular_z={angular_z}, duration_s={duration_s}')
                
                # 既存のスレッドがあれば停止リクエストを送る
                self.stop_requested = True
                if self.command_thread and self.command_thread.is_alive():
                    self.command_thread.join(timeout=1.0) # 少し待つ
                self.stop_requested = False # 新しいコマンドのためにリセット

                self.command_thread = threading.Thread(
                    target=self._execute_move_command,
                    args=(linear_x, angular_z, duration_s)
                )
                self.command_thread.start()

            else:
                self.get_logger().warn(f'Unknown command_type: {command_type}')

        except json.JSONDecodeError:
            self.get_logger().error(f'Failed to decode JSON: {msg.data}')
        except ValueError as e:
            self.get_logger().error(f'Invalid parameter type in JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'An unexpected error occurred: {e}')

    def _execute_move_command(self, linear_x, angular_z, duration_s):
        """指定された速度で指定時間動作し、その後停止する処理を別スレッドで実行"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        start_time = self.get_clock().now()
        end_time = start_time + Duration(seconds=duration_s)

        # 0.1秒ごとにコマンドをパブリッシュ
        while rclpy.ok() and self.get_clock().now() < end_time and not self.stop_requested:
            self.publisher_.publish(twist)
            time.sleep(0.1)
        
        # 停止コマンドをパブリッシュ
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)
        self.get_logger().info('Move command finished or stopped.')

    def on_shutdown(self):
        self.stop_requested = True
        if self.command_thread and self.command_thread.is_alive():
            self.command_thread.join(timeout=1.0)
        self.get_logger().info('Shutting down JSON Command Interpreter Node.')


def main(args=None):
    rclpy.init(args=args)
    node = JsonCommandInterpreter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()