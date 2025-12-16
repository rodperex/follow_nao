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
from nao_lola_sensor_msgs.msg import Sonar, Touch
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from sensor_msgs.msg import CameraInfo
# from .pid_controller import PIDController

class MotionControl(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')

        # Parameter: minimum distance to consider obstacle
        self.declare_parameter('avoidance_distance', 0.5)
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_linear_speed', 0.5)
        self.declare_parameter('max_angular_speed', 1.0)
        self.declare_parameter('optical_frame', 'CameraTop_optical_frame')
        self.declare_parameter('target_class', 'person')

        self.avoidance_distance = self.get_parameter('avoidance_distance').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_linear_speed = self.get_parameter('max_linear_speed').value
        self.max_angular_speed = self.get_parameter('max_angular_speed').value
        self.optical_frame = self.get_parameter('optical_frame').value
        self.target_class = self.get_parameter('target_class').value


        self.get_logger().info(f'Obstacle minimum distance={self.avoidance_distance}')

        # self.vrot_pid = PIDController(0.0, 1.0, 0.3, 1.0)

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            'camera_info',
            self.camera_info_callback,
            rclpy.qos.qos_profile_sensor_data
        )

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

        self.sub = self.create_subscription(
            Detection2DArray,
            'input_detection_2d',
            self.detection_callback,
            rclpy.qos.qos_profile_sensor_data
        )

        self.cmd_pub = self.create_publisher(Twist, 'vel', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.1, self.control_cycle)

        self.vel_rot = 0.0
        self.detection_frame = None
        self.detection = None
        self.configured = False
        self.stop = False

    def camera_info_callback(self, msg: CameraInfo):
        # The intrinsic matrix K is a 9-element array (row-major order)
        # K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.f_x = msg.k[0] # fx is K[0]
        self.c_x = msg.k[2] # cx is K[2]

        self.current_image_size = (msg.width, msg.height)
        self.get_logger().info(f'Got image of size: {msg.width}x{msg.height}')
        self.get_logger().info(f'Got camera intrinsics: fx={self.f_x:.2f}, cx={self.c_x:.2f}')
        self.configured = True
        self.destroy_subscription(self.camera_info_sub)


    def detection_callback(self, msg: Detection2DArray):
        if not msg.detections:
            return
        
        self.get_logger().debug(f'Received {len(msg.detections)} detections')

        for detection in msg.detections:
            if detection.results and detection.results[0].hypothesis.class_id == self.target_class:
                self.get_logger().debug(f'Selected detection of class {self.target_class}')
                self.detection = detection
                break
    
    def touch_callback(self, msg: Touch):
        if msg.head_front or msg.head_middle or msg.head_rear:
            if not self.stop:   
                self.get_logger().info('Touch detected on head, stopping robot')
                self.stop = True
            else:
                self.get_logger().info('Touch detected on head, resuming robot')
                self.stop = False

    def sonar_callback(self, msg: Sonar):

        self.get_logger().debug(f'Sonar distances - Left: {msg.left:.2f} m, Right: {msg.right:.2f} m')

        dis_left = msg.left
        dis_right = msg.right    
        avoid_left = False
        avoid_right = False
        vel_rot_avoid_left = 0.0
        vel_rot_avoid_right = 0.0
        vel_rot = 0.0
        
        if dis_left < self.avoidance_distance: # Obstacle on the left
            self.get_logger().debug(f'Obstacle detected on the LEFT at distance {dis_left:.2f} m')
            avoid_left = True
            # vel_rot_avoid_left = max(-self.max_angular_speed, self.avoidance_distance - dis_left, self.max_angular_speed)
            vel_rot_avoid_left = -self.max_angular_speed
            vel_rot = vel_rot_avoid_left

        if dis_right < self.avoidance_distance: # Obstacle on the right
            self.get_logger().debug(f'Obstacle detected on the RIGHT at distance {dis_right:.2f} m')
            avoid_right = True
            # vel_rot_avoid_right = min(self.max_angular_speed, dis_right - self.avoidance_distance, self.max_angular_speed)
            vel_rot_avoid_right = self.max_angular_speed
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

        if self.detection is None:
            self.get_logger().debug('No detection available')
            return
        
        if not self.configured:
            self.get_logger().warn('Camera info not yet received, cannot compute angles')
            return
        
        if self.stop:
            self.get_logger().info('Robot stopped due to touch sensor')
            return
        
        # Calculate angle relative to image center (positive left, negative right)
        x_pixel = self.detection.bbox.center.position.x
        self.get_logger().debug(f'Detection center x: {x_pixel:.2f}. BB width: {self.detection.bbox.size_x:.2f}')
        

        # Use camera intrinsics to compute angle
        f_x = self.f_x
        c_x = self.c_x

        pixel_offset_x = x_pixel - c_x
        angle = math.atan(pixel_offset_x / f_x)
        self.get_logger().debug(f'Detected {self.target_class} at angle {math.degrees(angle):.1f} degrees ({self.optical_frame})')

        x_optical = math.tan(angle)
        y_optical = 0.0  
        z_optical = 1.0 # Fixed distance of 1 meter. No depth info from 2D detection

        target_point = PointStamped()
        target_point.header = self.detection.header
        target_point.point.x = x_optical
        target_point.point.y = y_optical
        target_point.point.z = z_optical

        source_frame = self.optical_frame
        target_frame = self.base_frame
        # detection_time = self.detection.header.stamp

        if not self.tf_buffer.can_transform(target_frame, source_frame, rclpy.time.Time()):
            self.get_logger().warn(f"Waiting for transform {target_frame} -> {source_frame}")
            return
        try:
            self.get_logger().debug(f'Looking up transform from {source_frame} to {target_frame}')

            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )

            transformed_point = do_transform_point(target_point, transform)

            angle = math.atan2(transformed_point.point.y, transformed_point.point.x)

            # vel_rot = max(-self.max_angular_speed, min(self.vrot_pid.get_output(angle), self.max_angular_speed))
            vel_rot = self.max_angular_speed if angle > 0 else -self.max_angular_speed
            vel_lin = self.max_linear_speed

            self.get_logger().info(f'Objective ({self.target_class}) @ angle {math.degrees(angle):.1f} degrees in {target_frame} frame')

            if vel_rot * self.vel_rot > 0.0:  # Same direction, add obstacle avoidance
                vel_rot += self.vel_rot
            
            self.get_logger().debug(f'linear speed={vel_lin:.2f} m/s, angular speed={vel_rot:.2f} rad/s')

            vel = Twist()
            vel.linear.x = vel_lin
            vel.angular.z = vel_rot

            # self.cmd_pub.publish(vel)

        except Exception as e:
            self.get_logger().warn(f'Error in TF {source_frame} -> {target_frame}: {str(e)}')
         

def main(args=None):
    rclpy.init(args=args)
    node = MotionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
