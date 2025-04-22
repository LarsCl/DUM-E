#!/usr/bin/env python3
import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.action import ActionServer
from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from pprint import pprint
from scripts.uart_classes import ArduinoUART, ODriveUART # Your custom Python class

class HardwareInterfaceNode(Node):
    def __init__(self):
        super().__init__('hardware_interface_node')
        print("Hardware interface active")
        # Initialize your UART hardware interface
        self.arduino= ArduinoUART(port='/dev/ttyUSB0', baud_rate=115200)
        self.odrive1= ODriveUART(port='/dev/ttyUSB1', baud_rate=115200)
        self.odrive2= ODriveUART(port='/dev/ttyUSB2', baud_rate=115200)

        self.odrive1.arm()
        self.odrive2.arm()
        
        # Create an action server for FollowJointTrajectory
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self.trajectory_callback
        )

        # Publisher for joint states (to be used by robot_state_publisher)
        self.joint_state_pub = self.create_publisher(JointState, 'joint_states', 10)
        
        # Timer for periodically reading hardware state and publishing it
        self.create_timer(0.01, self.read_and_publish_state)
        self.get_logger().info("Hardware Interface Node initialized.")

    def trajectory_callback(self, goal_handle):
        """
        This function is called when a trajectory goal is received.
        It executes the trajectory by sending commands to the hardware.
        """
        self.get_logger().info("Executing trajectory command...")

        trajectory = goal_handle.request.trajectory

        # Process each point in the trajectory
        poses = trajectory.points[-1].positions
        poses[0] = poses[0] / np.pi * 1800
        poses[3] = poses[3] / np.pi * 1800
        poses[4] = poses[4] / np.pi * 1800
        poses[5] = poses[5] / np.pi * 1800
        print(poses)
        self.arduino.set_postition(int(poses[0]), int(poses[3]), int(poses[4]), int(poses[5]))
        for point in trajectory.points:
            positions = point.positions
            velocities = point.velocities
            print(f"we going to {point}")
            # Example: Send commands to hardware
            #self.arduino.send_joint_positions([positions[0], positions[3], positions[4], positions[5]])
            self.odrive_pos1 = (positions[1])/(2 * np.pi)*7.5
            self.odrive_pos2 = (positions[2])/(2 * np.pi)*7.5 - self.odrive_pos1
            self.odrive_vel1 = (velocities[1])/(2 * np.pi)*7.5
            self.odrive_vel2 = (velocities[2])/(2 * np.pi)*7.5 - self.odrive_vel1

            self.odrive1.move_pos(self.odrive_pos1, self.odrive_vel1)
            self.odrive2.move_pos(self.odrive_pos2, self.odrive_vel2)

            # Simulate execution delay (you can replace this with real-time checks)
            self.get_clock().sleep_for(rclpy.duration.Duration(seconds=0.1))

        # Mark the goal as succeeded
        goal_handle.succeed()

        # Return a result message
        result = FollowJointTrajectory.Result()
        self.get_logger().info("Trajectory execution completed.")
        return result

    def read_and_publish_state(self):
        # Read joint state from your hardware via UART
        arduino_pos_state = self.arduino.get_positions() # = degrees * 10
        arduino_vel_state = self.arduino.get_velocities() # = rpm * 10
        if arduino_pos_state or arduino_vel_state == [None, None, None, None]:
            for i in range(4):
                arduino_pos_state[i] = 0
                arduino_vel_state[i] = 0
        else:
            for i in range(4):
                arduino_pos_state[i] = ((arduino_pos_state[i] * np.pi) / 1800)
                arduino_vel_state[i] = ((arduino_vel_state[i] / 600) * 2 * np.pi)
        odrive1_pos_state = (2 * np.pi * self.odrive1.get_joint_position()) / 7.5
        odrive1_vel_state = (2 * np.pi * self.odrive1.get_joint_velocity()) / 7.5
        odrive2_pos_state = (2 * np.pi * self.odrive2.get_joint_position()) / 7.5 + odrive1_pos_state
        odrive2_vel_state = (2 * np.pi * self.odrive2.get_joint_velocity()) / 7.5 + odrive1_vel_state

        joint_positions =  [arduino_pos_state[0], odrive1_pos_state, odrive2_pos_state, arduino_pos_state[1], arduino_pos_state[2], arduino_pos_state[3]]
        joint_velocities = [arduino_vel_state[0], odrive1_vel_state, odrive2_vel_state, arduino_vel_state[1], arduino_vel_state[2], arduino_vel_state[3]]
        # Create a JointState message and publish it
        js_msg = JointState()
        js_msg.header.stamp = self.get_clock().now().to_msg()
        js_msg.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6']
        js_msg.position = joint_positions
        js_msg.velocity = joint_velocities
        self.joint_state_pub.publish(js_msg)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Hardware Interface Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
