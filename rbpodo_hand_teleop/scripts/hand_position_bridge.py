#!/usr/bin/env python3
"""
Bridge to receive 3D hand position from Mac and publish to ROS 2.
Receives JSON data from mediapipe_hand_server.py and publishes as PointStamped.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Bool
import socket
import struct
import json


class HandPositionBridge(Node):
    def __init__(self):
        super().__init__('hand_position_bridge')
        
        # Parameters
        self.declare_parameter('mac_ip', 'host.internal')  # OrbStack's host IP
        self.declare_parameter('mac_port', 9998)  # Different port for hand data
        self.declare_parameter('position_topic', '/hand/position')
        self.declare_parameter('detected_topic', '/hand/detected')
        
        self.mac_ip = self.get_parameter('mac_ip').value
        self.mac_port = self.get_parameter('mac_port').value
        position_topic = self.get_parameter('position_topic').value
        detected_topic = self.get_parameter('detected_topic').value
        
        self.get_logger().info(f'Connecting to hand server at {self.mac_ip}:{self.mac_port}')
        self.get_logger().info(f'Publishing position to {position_topic}')
        
        # Publishers
        self.position_pub = self.create_publisher(PointStamped, position_topic, 10)
        self.detected_pub = self.create_publisher(Bool, detected_topic, 10)
        
        # Socket connection
        self.sock = None
        
        # Timer to receive and publish hand data at high rate
        self.timer = self.create_timer(0.02, self.receive_and_publish)  # 50 Hz
        
        self.connect_to_mac()
        self.connection_warned = False
    
    def connect_to_mac(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.mac_ip, self.mac_port))
            self.sock.settimeout(0.1)
            self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            self.get_logger().info('Connected to MediaPipe hand server!')
            self.connection_warned = False
        except Exception as e:
            if not self.connection_warned:
                self.get_logger().warn(f'Could not connect to Mac: {e}')
                self.get_logger().info('Make sure mediapipe_hand_server.py is running on your Mac')
                self.connection_warned = True
            self.sock = None
    
    def receive_and_publish(self):
        if self.sock is None:
            self.connect_to_mac()
            return
        
        try:
            # Receive data size
            size_data = self.recvall(4)
            if size_data is None:
                return
            
            size = struct.unpack(">I", size_data)[0]
            
            # Receive JSON data
            json_data = self.recvall(size)
            if json_data is None:
                return
            
            # Parse hand data
            hand_data = json.loads(json_data.decode('utf-8'))
            
            # Publish detected status
            detected_msg = Bool()
            detected_msg.data = hand_data.get('detected', False)
            self.detected_pub.publish(detected_msg)
            
            # Publish position if detected
            if hand_data.get('detected', False):
                pos_msg = PointStamped()
                pos_msg.header.stamp = self.get_clock().now().to_msg()
                pos_msg.header.frame_id = 'hand'
                pos_msg.point.x = hand_data.get('x', 0.0)  # Left/right
                pos_msg.point.y = hand_data.get('y', 0.0)  # Up/down
                pos_msg.point.z = hand_data.get('z', 0.0)  # Forward/back
                self.position_pub.publish(pos_msg)
                
        except socket.timeout:
            pass
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'JSON decode error: {e}')
        except Exception as e:
            self.get_logger().warn(f'Connection lost: {e}')
            self.sock = None
    
    def recvall(self, size):
        data = b''
        while len(data) < size:
            try:
                packet = self.sock.recv(size - len(data))
                if not packet:
                    return None
                data += packet
            except socket.timeout:
                return None
        return data


def main(args=None):
    rclpy.init(args=args)
    node = HandPositionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
