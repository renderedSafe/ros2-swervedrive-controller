# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist
import math
from pySerialTransfer import pySerialTransfer as txfer

import time
import random

class RobotStateCommand:
    def __init__(self, turn_angle:float=0, drive_power:float=0 ) -> None:
        self.turn_angle = turn_angle
        self.drive_power = drive_power
        self.last_increment_time = time.time()
        self.angle_increment_interval = 0.05  # time interval in seconds between angle updates with the increment_angle method
        self.angle_increment_multiplier = 1
        self.max_angle_deflection = 60

    def increment_angle(self, inc_val: float):
        #not, for early exit. most calls should exit here
        if not ((time.time() - self.last_increment_time) > self.angle_increment_interval):
            pass
        elif abs(self.angle_increment_multiplier * inc_val + self.turn_angle) > self.max_angle_deflection:
            self.turn_angle = math.copysign(self.max_angle_deflection, inc_val)
        else:
            self.turn_angle += self.angle_increment_multiplier * inc_val

         
class ArduinoJoySubscriber(Node):

    def __init__(self):
        super().__init__('arduino_joy_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.robot_state_command = RobotStateCommand()
        self.robot_state_command.angle_increment_multiplier = 5
        self.link = txfer.SerialTransfer('/dev/ttyACM0')
        self.link.open()
        time.sleep(2)

    def listener_callback(self, msg: Twist):
        self.robot_state_command.increment_angle(msg.angular.z)
        self.robot_state_command.drive_power = msg.linear.x
        self.get_logger().info(f"Robot angle: {self.robot_state_command.turn_angle}")
        self.get_logger().info(f"Robot drive power: {self.robot_state_command.drive_power}")

        send_size = 0
        send_size = self.link.tx_obj(self.robot_state_command.turn_angle, send_size, val_type_override='f')
        send_size = self.link.tx_obj(self.robot_state_command.drive_power, send_size, val_type_override='f')
        self.link.send(send_size)



def main(args=None):
    rclpy.init(args=args)

    arduino_joy_rest = ArduinoJoySubscriber()

    rclpy.spin(arduino_joy_rest)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    arduino_joy_rest.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
