#!/usr/bin/env python3
"""
3D Hand Teleoperation with Streaming Servo Control (Python version)

This node uses eval service to call servo_l for real-time control.
Your hand position in a 3D cube directly maps to the robot TCP position.

For debugging: Run with enable_debug:=true to see detailed output.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool, Float64MultiArray
from rbpodo_msgs.msg import SystemState
import math


class HandTeleopStreaming(Node):
    def __init__(self):
        super().__init__('hand_teleop_streaming')
        
        # Parameters
        self.declare_parameter('workspace_size', 0.05)
        self.declare_parameter('control_rate', 30.0)
        self.declare_parameter('enable_debug', True)
        
        # Servo control parameters (smoother defaults)
        self.declare_parameter('servo_t1', 0.08)
        self.declare_parameter('servo_t2', 0.15)
        self.declare_parameter('servo_gain', 0.6)
        self.declare_parameter('servo_alpha', 0.2)
        self.declare_parameter('smoothing_factor', 0.3)  # Position smoothing
        
        self.workspace_size = self.get_parameter('workspace_size').value
        self.control_rate = self.get_parameter('control_rate').value
        self.enable_debug = self.get_parameter('enable_debug').value
        self.servo_t1 = self.get_parameter('servo_t1').value
        self.servo_t2 = self.get_parameter('servo_t2').value
        self.servo_gain = self.get_parameter('servo_gain').value
        self.servo_alpha = self.get_parameter('servo_alpha').value
        self.smoothing_factor = self.get_parameter('smoothing_factor').value
        
        # Safety clamp
        if self.workspace_size > 0.15:
            self.get_logger().warn('Workspace clamped to 15cm MAX for safety!')
            self.workspace_size = 0.15
        
        self.get_logger().warn('=' * 50)
        self.get_logger().warn('3D HAND TELEOPERATION - STREAMING MODE')
        self.get_logger().warn('=' * 50)
        self.get_logger().info(f'Workspace: +/- {self.workspace_size * 100:.1f} cm cube')
        self.get_logger().info(f'Control rate: {self.control_rate:.0f} Hz')
        
        # State
        self.tcp_initialized = False
        self.initial_pos = [0.0] * 6  # x, y, z, rx, ry, rz
        self.target_pos = [0.0] * 3   # Smoothed target x, y, z
        self.hand_pos = [0.0, 0.0, 0.0]  # x, y, z normalized [-1, 1]
        self.hand_detected = False
        self.last_hand_time = self.get_clock().now()
        self.streaming_active = False
        self.hand_lost_count = 0
        
        # Subscribers
        self.state_sub = self.create_subscription(
            SystemState,
            '/rbpodo_hardware/system_state',
            self.state_callback,
            10
        )
        
        self.hand_pos_sub = self.create_subscription(
            PointStamped,
            '/hand/position',
            self.hand_position_callback,
            10
        )
        
        self.hand_detected_sub = self.create_subscription(
            Bool,
            '/hand/detected',
            self.hand_detected_callback,
            10
        )
        
        # Publisher for servo commands
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.servo_pub = self.create_publisher(
            Float64MultiArray, '/rbpodo_hardware/servo_cartesian', qos)
        
        # Control timer
        period = 1.0 / self.control_rate
        self.control_timer = self.create_timer(period, self.control_callback)
        
        self.get_logger().info('Waiting for robot SystemState and hand position...')
    
    def state_callback(self, msg):
        if not self.tcp_initialized:
            self.initial_pos = list(msg.tcp_pos)
            self.target_pos = [self.initial_pos[0], self.initial_pos[1], self.initial_pos[2]]
            self.tcp_initialized = True
            
            self.get_logger().info('=' * 50)
            self.get_logger().info('Initial TCP from robot:')
            self.get_logger().info(f'  Position: ({self.initial_pos[0]:.3f}, {self.initial_pos[1]:.3f}, {self.initial_pos[2]:.3f}) m')
            rx_deg = math.degrees(self.initial_pos[3])
            ry_deg = math.degrees(self.initial_pos[4])
            rz_deg = math.degrees(self.initial_pos[5])
            self.get_logger().info(f'  Orientation: ({rx_deg:.1f}, {ry_deg:.1f}, {rz_deg:.1f}) deg')
            self.get_logger().info(f'Workspace: +/- {self.workspace_size * 100:.1f} cm 3D cube')
            self.get_logger().info(f'Smoothing: {self.smoothing_factor:.2f}')
            self.get_logger().info('=' * 50)
            self.get_logger().info('Move your hand in 3D space to control the robot!')
    
    def hand_position_callback(self, msg):
        self.hand_pos[0] = msg.point.x  # Left/right
        self.hand_pos[1] = msg.point.y  # Up/down
        self.hand_pos[2] = msg.point.z  # Depth
        self.last_hand_time = self.get_clock().now()
        self.hand_lost_count = 0
    
    def hand_detected_callback(self, msg):
        self.hand_detected = msg.data
        if not self.hand_detected:
            self.hand_lost_count += 1
    
    def control_callback(self):
        if not self.tcp_initialized:
            return
        
        # Check hand visibility (detected and recent)
        time_since_hand = (self.get_clock().now() - self.last_hand_time).nanoseconds / 1e9
        hand_visible = self.hand_detected and time_since_hand < 0.5
        
        if not hand_visible:
            if self.streaming_active and self.hand_lost_count == 5:
                self.get_logger().info('Hand lost - robot holds position')
                self.streaming_active = False
            return
        
        # Calculate target position
        # USER-REQUESTED MAPPING:
        #   Hand left/right (X) -> Robot X (forward/back)
        #   Hand depth (Z)      -> Robot Y: closer = Y-, farther = Y+
        #   Hand up/down (Y)    -> Robot Z
        hx, hy, hz = self.hand_pos
        
        offset_x = -hx * self.workspace_size  # Hand left = robot forward, hand right = robot back
        offset_y = -hz * self.workspace_size  # Hand closer = Y-, hand farther = Y+
        offset_z = hy * self.workspace_size   # Hand up = robot up
        
        # Clamp to workspace
        offset_x = max(-self.workspace_size, min(self.workspace_size, offset_x))
        offset_y = max(-self.workspace_size, min(self.workspace_size, offset_y))
        offset_z = max(-self.workspace_size, min(self.workspace_size, offset_z))
        
        # Raw target position
        raw_x = self.initial_pos[0] + offset_x
        raw_y = self.initial_pos[1] + offset_y
        raw_z = self.initial_pos[2] + offset_z
        
        # Apply smoothing filter (exponential moving average)
        s = self.smoothing_factor
        self.target_pos[0] = s * raw_x + (1.0 - s) * self.target_pos[0]
        self.target_pos[1] = s * raw_y + (1.0 - s) * self.target_pos[1]
        self.target_pos[2] = s * raw_z + (1.0 - s) * self.target_pos[2]
        
        target_x, target_y, target_z = self.target_pos
        
        # Safety check
        dist = math.sqrt(offset_x**2 + offset_y**2 + offset_z**2)
        if dist > self.workspace_size * 1.8:
            self.get_logger().error(f'SAFETY: Target {dist:.3f}m exceeds bounds!')
            return
        
        # Send servo command via topic
        self.send_servo_command(
            target_x, target_y, target_z,
            self.initial_pos[3], self.initial_pos[4], self.initial_pos[5]
        )
        
        if not self.streaming_active:
            self.get_logger().info('Streaming control started!')
            self.streaming_active = True
        
        if self.enable_debug:
            self.get_logger().info(
                f'Hand({hx:+.2f},{hy:+.2f},{hz:+.2f}) -> TCP({target_x:.3f},{target_y:.3f},{target_z:.3f}) '
                f'off({offset_x*100:.1f},{offset_y*100:.1f},{offset_z*100:.1f})cm',
                throttle_duration_sec=0.2
            )
    
    def send_servo_command(self, x, y, z, rx, ry, rz):
        """Send servo command via topic."""
        # Publish Float64MultiArray with [x, y, z, rx, ry, rz] in meters and radians
        msg = Float64MultiArray()
        msg.data = [x, y, z, rx, ry, rz]
        self.servo_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HandTeleopStreaming()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
