#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import UInt32
from std_msgs.msg import Int32

import time

import numpy as np

import utility as ut

GANTRY_CRANE_NODE_NAME = "simulator_publisher_node"

TROLLEY_POSITION_TOPIC_NAME = "trolley_position"
CABLE_LENGTH_TOPIC_NAME = "cable_length"
SWAY_ANGLE_TOPIC_NAME = "sway_angle"
TROLLEY_MOTOR_VOLTAGE_TOPIC_NAME = "trolley_motor_voltage"
HOIST_MOTOR_VOLTAGE_TOPIC_NAME = "hoist_motor_voltage"

CONTROLLER_COMMAND_TOPIC_NAME = "controller_command"
TROLLEY_MOTOR_PWM_TOPIC_NAME = "trolley_motor_PWM"
HOIST_MOTOR_PWM_TOPIC_NAME = "hoist_motor_PWM"

TIME_STEP = 0.02

class SimulatorPublisherNode(Node):
    def __init__(self):
        super().__init__(GANTRY_CRANE_NODE_NAME)
        self.trolley_position_publisher = self.create_publisher(Float32, TROLLEY_POSITION_TOPIC_NAME, 10)
        self.cable_length_publisher = self.create_publisher(Float32, CABLE_LENGTH_TOPIC_NAME, 10)
        self.sway_angle_publisher = self.create_publisher(Float32, SWAY_ANGLE_TOPIC_NAME, 10)
        self.trolley_motor_voltage_publisher = self.create_publisher(Float32, TROLLEY_MOTOR_VOLTAGE_TOPIC_NAME, 10)
        self.hoist_motor_voltage_publisher = self.create_publisher(Float32, HOIST_MOTOR_VOLTAGE_TOPIC_NAME, 10)

        self.controller_command_publisher = self.create_publisher(UInt32, CONTROLLER_COMMAND_TOPIC_NAME, 10)
        self.trolley_motor_PWM_publisher = self.create_publisher(Int32, TROLLEY_MOTOR_PWM_TOPIC_NAME, 10)
        self.hoist_motor_PWM_publisher = self.create_publisher(Int32, HOIST_MOTOR_PWM_TOPIC_NAME, 10)

        self.timer = self.create_timer(TIME_STEP, self.timer_callback)

        self.trolley_position = 0.0
        self.cable_length = 0.0
        self.sway_angle = 0.0
        self.trolley_motor_voltage = 0.0
        self.hoist_motor_voltage = 0.0

        self.controller_command = 0
        self.trolley_motor_PWM = 0
        self.hoist_motor_PWM = 0

    def timer_callback(self):
        self.trolley_position_publisher.publish(Float32(data=self.trolley_position))
        self.cable_length_publisher.publish(Float32(data=self.cable_length))
        self.sway_angle_publisher.publish(Float32(data=self.sway_angle))
        self.trolley_motor_voltage_publisher.publish(Float32(data=self.trolley_motor_voltage))
        self.hoist_motor_voltage_publisher.publish(Float32(data=self.hoist_motor_voltage))

        self.controller_command_publisher.publish(UInt32(data=self.controller_command))
        self.trolley_motor_PWM_publisher.publish(Int32(data=self.trolley_motor_PWM))
        self.hoist_motor_PWM_publisher.publish(Int32(data=self.hoist_motor_PWM))

        self.trolley_position = np.random.randn()
        self.cable_length = np.random.randn()
        self.sway_angle = np.random.randn()
        self.trolley_motor_voltage = np.random.randn()
        self.hoist_motor_voltage = np.random.randn()

        # self.controller_command = np.random.randint(-1023, 1023)
        self.trolley_motor_PWM = np.random.randint(-1023, 1023)
        self.hoist_motor_PWM = np.random.randint(-1023, 1023)
    
    def simulate_gantry_crane(self):
        pass

def main(args=None):
    rclpy.init(args=args)

    simulator_publisher_node = SimulatorPublisherNode()

    rclpy.spin(simulator_publisher_node)

    simulator_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()