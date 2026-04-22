#!/usr/bin/env python3
"""
Bridge to receive camera images from Mac and publish to ROS 2.
Run camera_server.py on your Mac first.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import socket
import pickle
import struct
import numpy as np


class MacCameraBridge(Node):
    def __init__(self):
        super().__init__('mac_camera_bridge')
        
        # Parameters
        self.declare_parameter('mac_ip', 'host.internal')  # OrbStack's host IP
        self.declare_parameter('mac_port', 9999)
        self.declare_parameter('image_topic', '/camera/image_raw')
        
        mac_ip = self.get_parameter('mac_ip').value
        mac_port = self.get_parameter('mac_port').value
        image_topic = self.get_parameter('image_topic').value
        
        self.get_logger().info(f'Connecting to Mac camera at {mac_ip}:{mac_port}')
        self.get_logger().info(f'Publishing to {image_topic}')
        
        # Publisher
        self.publisher = self.create_publisher(Image, image_topic, 10)
        self.bridge = CvBridge()
        
        # Socket connection
        self.sock = None
        self.mac_ip = mac_ip
        self.mac_port = mac_port
        
        # Timer to receive and publish images
        self.timer = self.create_timer(0.033, self.receive_and_publish)  # ~30 fps
        
        self.connect_to_mac()
    
    def connect_to_mac(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.mac_ip, self.mac_port))
            self.sock.settimeout(1.0)
            self.get_logger().info('Connected to Mac camera!')
        except Exception as e:
            self.get_logger().warn(f'Could not connect to Mac: {e}')
            self.get_logger().info('Make sure camera_server.py is running on your Mac')
            self.sock = None
    
    def receive_and_publish(self):
        if self.sock is None:
            self.connect_to_mac()
            return
        
        try:
            # Receive frame size
            size_data = self.recvall(4)
            if size_data is None:
                return
            
            size = struct.unpack(">L", size_data)[0]
            
            # Receive frame data
            frame_data = self.recvall(size)
            if frame_data is None:
                return
            
            # Decode frame
            frame = pickle.loads(frame_data)
            
            # Validate frame
            if frame is None or not hasattr(frame, 'shape') or len(frame.shape) < 2:
                self.get_logger().warn('Invalid frame received, skipping')
                return
            
            # Convert to ROS message and publish
            try:
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'camera'
                self.publisher.publish(msg)
            except Exception as e:
                self.get_logger().warn(f'Failed to convert frame: {e}')
                return
            
        except socket.timeout:
            pass
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
    node = MacCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
