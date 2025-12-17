# Copyright 2025 Rodrigo Pérez-Rodríguez
#
# Licensed under the Apache License, Version 2.0 (the "License");
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.

import rclpy
import math
from rclpy.node import Node
from nao_lola_sensor.msg import Sonar, Touch
from geometry_msgs.msg import Twist
from tf2_ros import Buffer, TransformListener
# from .pid_controller import PIDController

class MotionControl3D(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        # Parameter: minimum distance to consider obstacle
        self.declare_parameter('min_distance', 1.0)
        self.declare_parameter('avoidance_distance', 0.5)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('target_frame', 'target')
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)

        self.min_distance = self.get_parameter('min_distance').value
        self.avoidance_distance = self.get_parameter('avoidance_distance').value
        self.base_frame = self.get_parameter('base_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value


        self.get_logger().info(f'Obstacle minimum distance={self.avoidance_distance}')

        # self.vlin_pid = PIDController(0.0, 1.0, 0.0, 0.7)
        # self.vrot_pid = PIDController(0.0, 1.0, 0.3, 1.0)

        self.sonar_sub = self.create_subscription(
            Sonar,
            'sonar',
            self.sonar_callback,
            10
        )

        self.touch_sub = self.create_subscription(
            Touch,
            'touch',
            self.touch_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, 'vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_cycle)

        self.vel_rot = 0.0

        self.stop = False


    def touch_callback(self, msg: Touch):
        if msg.head_front or msg.head_middle or msg.head_rear:
            if not self.stop:   
                self.get_logger().info('Touch detected on head, stopping robot')
                self.stop = True
            else:
                self.get_logger().info('Touch detected on head, resuming robot')
                self.stop = False

    def sonar_callback(self, msg: Sonar):

        dis_left = msg.left
        dis_right = msg.right    
        avoid_left = False
        avoid_right = False
        vel_rot_avoid_left = 0.0
        vel_rot_avoid_right = 0.0
        vel_rot = 0.0
        
        
        if dis_left < self.avoidance_distance: # Obstacle on the left
            avoid_left = True
            vel_rot_avoid_left = max(-self.max_angular_speed, self.avoidance_distance - dis_left, self.max_angular_speed)
            vel_rot = vel_rot_avoid_left

        if dis_right < self.avoidance_distance: # Obstacle on the right
            avoid_right = True
            vel_rot_avoid_right = min(self.max_angular_speed, dis_right - self.avoidance_distance, self.max_angular_speed)
            vel_rot = vel_rot_avoid_right
        
        if avoid_left and avoid_right: # Obstacles on both sides
            if dis_left < dis_right:
                vel_rot = vel_rot_avoid_left
            elif dis_right < dis_left:
                vel_rot = vel_rot_avoid_right
            else:
                vel_rot = 0.0

        self.vel_rot = vel_rot

    def control_cycle(self):

        if not self.tf_buffer.can_transform(self.base_frame, self.target_frame, rclpy.time.Time()):
            self.get_logger().warn('Waiting for transform base_footprint -> target')
            return
        
        if self.stop:
            self.get_logger().debug('Robot stopped due to touch sensor')
            self.cmd_pub.publish(Twist())  # Publish zero velocities
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.target_frame, rclpy.time.Time())

            x = tf.transform.translation.x
            y = tf.transform.translation.y

            angle = math.atan2(y, x)
            dist = math.sqrt(x ** 2 + y ** 2)

            # vel_rot = max(-2.0, min(self.vrot_pid.get_output(angle), 2.0))
            # vel_lin = max(-1.0, min(self.vlin_pid.get_output(dist - self.min_distance), 1.0))

            vel_rot = self.max_angular_speed if angle > 0 else -self.max_angular_speed
            vel_lin = self.max_linear_speed if dist > self.min_distance else 0.0

            if vel_rot * self.vel_rot > 0.0:  # Same direction, add obstacle avoidance
                vel_rot += self.vel_rot
            
            if dist < self.min_distance:
                vel_lin = 0.0

            vel = Twist()
            vel.linear.x = vel_lin
            vel.angular.z = vel_rot

            self.cmd_pub.publish(vel)

        except Exception as e:
            self.get_logger().warn(f'Error in TF {self.base_frame} -> {self.target_frame}: {str(e)}')
         

def main(args=None):
    rclpy.init(args=args)
    node = MotionControl3D()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
