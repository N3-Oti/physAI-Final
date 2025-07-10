#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String # JSON文字列を受信するために使用
import json
import time

class JsonCommandInterpreter(Node):
    def __init__(self):
        super().__init__('json_command_interpreter')
        self.subscription = self.create_subscription(
            String,
            '/robot_commands_json', # JSONコマンドを受信するトピック
            self.json_command_callback,
            10)
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.get_logger().info('JSON Command Interpreter Node (turtlesim対応) has been started.')

    def json_command_callback(self, msg):
        self.get_logger().info(f'Received JSON: "{msg.data}"')
        try:
            commands = json.loads(msg.data)
            if not isinstance(commands, list):
                commands = [commands]
            for idx, command in enumerate(commands):
                command_type = command.get('command_type')
                parameters = command.get('parameters', {})
                if command_type == 'turn':
                    angle_deg = float(parameters.get('angle_deg', 0.0))
                    # turtlesim: 1.0 rad/s で回すと仮定し、角度から時間を計算
                    angular_speed = 1.0  # [rad/s]
                    angle_rad = angle_deg * 3.14159265 / 180.0
                    duration = abs(angle_rad / angular_speed)
                    twist = Twist()
                    twist.angular.z = angular_speed if angle_rad > 0 else -angular_speed
                    self.get_logger().info(f'[{idx}] turn: {angle_deg}deg ({angle_rad:.2f}rad), duration={duration:.2f}s')
                    self._publish_for_duration(twist, duration)
                elif command_type == 'move':
                    distance = float(parameters.get('distance_m', 0.0))
                    linear_speed = 1.0  # [m/s]
                    duration = abs(distance / linear_speed)
                    twist = Twist()
                    twist.linear.x = linear_speed if distance > 0 else -linear_speed
                    self.get_logger().info(f'[{idx}] move: {distance}m, duration={duration:.2f}s')
                    self._publish_for_duration(twist, duration)
                elif command_type == 'stop':
                    duration = float(parameters.get('duration_s', 0.5))
                    twist = Twist()  # 0速度
                    self.get_logger().info(f'[{idx}] stop: {duration}s')
                    self._publish_for_duration(twist, duration)
                else:
                    self.get_logger().warn(f'Unknown command_type: {command_type}')
        except Exception as e:
            self.get_logger().error(f'Error processing JSON commands: {e}')

    def _publish_for_duration(self, twist, duration):
        start_time = time.time()
        rate = 10  # 10Hz
        while rclpy.ok() and (time.time() - start_time) < duration:
            self.publisher_.publish(twist)
            time.sleep(1.0 / rate)
        # 最後に必ず停止
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)


def main(args=None):
    rclpy.init(args=args)
    node = JsonCommandInterpreter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()