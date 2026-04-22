#!/usr/bin/env python3
"""
MediaPipe Hand Tracking Server for Mac.
Streams 3D hand position to OrbStack for real-time robot teleoperation.

Features:
- True 3D hand tracking using MediaPipe
- Depth estimation from hand size
- High-rate streaming (30-60Hz)
- Normalized coordinates in [-1, 1] cube

Usage:
    pip3 install opencv-python mediapipe numpy
    python3 mediapipe_hand_server.py

Then in OrbStack, run (hand stack off unless you pass enable_hand_teleop):
    ros2 launch rbpodo_bringup rbpodo_hand_teleop_streaming.launch.py enable_hand_teleop:=true use_mac_camera:=true ...
"""

import cv2
import numpy as np
import socket
import struct
import json
import time
import argparse

# MediaPipe import with better error handling
# MediaPipe 0.10+ uses a new API structure
MP_AVAILABLE = False
mp = None
mp_hands = None
mp_drawing = None

try:
    import mediapipe as mp
    
    # Try different import methods for different MediaPipe versions
    try:
        # Method 1: Standard import (MediaPipe < 0.10)
        mp_hands = mp.solutions.hands
        mp_drawing = mp.solutions.drawing_utils
        MP_AVAILABLE = True
        print("Using MediaPipe legacy API (mp.solutions)")
    except AttributeError:
        try:
            # Method 2: Direct import from python.solutions (MediaPipe 0.10+)
            from mediapipe.python.solutions import hands as mp_hands_module
            from mediapipe.python.solutions import drawing_utils as mp_drawing_module
            mp_hands = mp_hands_module
            mp_drawing = mp_drawing_module
            MP_AVAILABLE = True
            print("Using MediaPipe 0.10+ API (mediapipe.python.solutions)")
        except ImportError as e1:
            try:
                # Method 3: Try importlib for dynamic loading
                import importlib
                hands_module = importlib.import_module('mediapipe.python.solutions.hands')
                drawing_module = importlib.import_module('mediapipe.python.solutions.drawing_utils')
                mp_hands = hands_module
                mp_drawing = drawing_module
                MP_AVAILABLE = True
                print("Using MediaPipe via importlib")
            except Exception as e2:
                # Diagnostic: show what's available
                print(f"ERROR: Could not find MediaPipe solutions module")
                print(f"  Method 2 error: {e1}")
                print(f"  Method 3 error: {e2}")
                print(f"MediaPipe version: {getattr(mp, '__version__', 'unknown')}")
                print(f"MediaPipe location: {mp.__file__ if hasattr(mp, '__file__') else 'unknown'}")
                print(f"Available top-level attributes: {[x for x in dir(mp) if not x.startswith('_')][:10]}")
                
                # Try to find solutions in submodules
                try:
                    import os
                    mp_path = os.path.dirname(mp.__file__) if hasattr(mp, '__file__') else None
                    if mp_path:
                        print(f"MediaPipe path: {mp_path}")
                        python_path = os.path.join(mp_path, 'python')
                        if os.path.exists(python_path):
                            print(f"Found python subdirectory: {python_path}")
                except:
                    pass
                    
                MP_AVAILABLE = False
                    
except ImportError:
    print("ERROR: MediaPipe not installed!")
    print("Install with: pip3 install mediapipe")
    MP_AVAILABLE = False
except Exception as e:
    print(f"ERROR: MediaPipe import failed: {e}")
    print("Try: pip3 install --upgrade mediapipe")
    MP_AVAILABLE = False


class HandTracker:
    """3D Hand tracking using MediaPipe."""
    
    def __init__(self, min_detection_confidence=0.7, min_tracking_confidence=0.7):
        global mp_hands, mp_drawing
        
        if not MP_AVAILABLE or mp_hands is None or mp_drawing is None:
            raise ImportError("MediaPipe is not available. Install with: pip3 install mediapipe")
        
        try:
            self.mp_hands = mp_hands
            self.mp_drawing = mp_drawing
            self.hands = self.mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=min_detection_confidence,
                min_tracking_confidence=min_tracking_confidence
            )
        except AttributeError as e:
            print(f"ERROR: MediaPipe API issue: {e}")
            print("Try reinstalling: pip3 install --upgrade mediapipe")
            raise
        
        # Calibration for depth estimation
        # Reference hand size at known distance (normalized)
        self.ref_hand_size = 0.25  # Approximate hand span at "neutral" distance
        
        # Smoothing filters
        self.alpha = 0.4  # Exponential smoothing factor (higher = more responsive)
        self.filtered_pos = None
        
    def process_frame(self, frame):
        """
        Process a frame and return 3D hand position.
        
        Returns:
            dict with keys: detected, x, y, z, confidence
            x, y, z are in range [-1, 1]
        """
        # Convert BGR to RGB
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)
        
        if not results.multi_hand_landmarks:
            return {
                'detected': False,
                'x': 0.0, 'y': 0.0, 'z': 0.0,
                'confidence': 0.0
            }
        
        # Get first hand
        hand = results.multi_hand_landmarks[0]
        
        # Extract key landmarks
        wrist = hand.landmark[self.mp_hands.HandLandmark.WRIST]
        middle_mcp = hand.landmark[self.mp_hands.HandLandmark.MIDDLE_FINGER_MCP]
        index_tip = hand.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
        pinky_mcp = hand.landmark[self.mp_hands.HandLandmark.PINKY_MCP]
        
        # Calculate palm center (average of wrist and middle MCP)
        palm_x = (wrist.x + middle_mcp.x) / 2
        palm_y = (wrist.y + middle_mcp.y) / 2
        
        # Estimate depth from hand size
        # Use distance between wrist and middle MCP, and index MCP to pinky MCP
        hand_height = np.sqrt((middle_mcp.x - wrist.x)**2 + (middle_mcp.y - wrist.y)**2)
        hand_width = np.sqrt((pinky_mcp.x - hand.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].x)**2 + 
                            (pinky_mcp.y - hand.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP].y)**2)
        hand_size = (hand_height + hand_width) / 2
        
        # Depth from hand size (larger hand = closer)
        # Also incorporate MediaPipe's z-coordinate (relative depth)
        if hand_size > 0.01:
            depth_from_size = (hand_size / self.ref_hand_size - 1.0)  # Positive = closer
        else:
            depth_from_size = 0.0
        
        # Combine with MediaPipe's z coordinate (wrist.z is negative when closer)
        mp_depth = -wrist.z * 2.0  # Scale factor
        
        # Weighted combination
        depth = 0.6 * depth_from_size + 0.4 * mp_depth
        
        # Convert to [-1, 1] range
        # X: left-right (MediaPipe x: 0=left, 1=right, invert for natural feel)
        x = -(palm_x * 2.0 - 1.0)  # Invert so moving hand right moves robot right from user's view
        
        # Y: up-down (MediaPipe y: 0=top, 1=bottom, invert for natural feel)
        y = -(palm_y * 2.0 - 1.0)  # Invert so raising hand moves robot up
        
        # Z: depth (closer = positive)
        z = np.clip(depth, -1.0, 1.0)
        
        # Apply exponential smoothing
        raw_pos = np.array([x, y, z])
        if self.filtered_pos is None:
            self.filtered_pos = raw_pos
        else:
            self.filtered_pos = self.alpha * raw_pos + (1 - self.alpha) * self.filtered_pos
        
        return {
            'detected': True,
            'x': float(self.filtered_pos[0]),
            'y': float(self.filtered_pos[1]),
            'z': float(self.filtered_pos[2]),
            'confidence': float(min(results.multi_handedness[0].classification[0].score, 1.0))
        }
    
    def draw_landmarks(self, frame, results=None):
        """Draw hand landmarks on frame for visualization."""
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                self.mp_drawing.draw_landmarks(
                    frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)
        return frame


def main():
    parser = argparse.ArgumentParser(description='MediaPipe Hand Tracking Server')
    parser.add_argument('--port', type=int, default=9998, help='Server port (default: 9998)')
    parser.add_argument('--camera', type=int, default=0, help='Camera index (default: 0)')
    parser.add_argument('--width', type=int, default=640, help='Frame width')
    parser.add_argument('--height', type=int, default=480, help='Frame height')
    parser.add_argument('--fps', type=int, default=30, help='Target FPS')
    parser.add_argument('--no-preview', action='store_true', help='Disable preview window')
    args = parser.parse_args()
    
    # Initialize camera
    print(f"Opening camera {args.camera}...")
    cap = cv2.VideoCapture(args.camera)
    
    if not cap.isOpened():
        print("ERROR: Could not open camera!")
        print("Make sure camera access is allowed in System Settings > Privacy & Security > Camera")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    
    # Test capture
    ret, frame = cap.read()
    if not ret:
        print("ERROR: Could not read from camera!")
        return
    
    print(f"Camera opened: {frame.shape[1]}x{frame.shape[0]}")
    
    # Check MediaPipe availability
    if not MP_AVAILABLE:
        print("\n" + "="*60)
        print("ERROR: MediaPipe solutions API not found!")
        print("="*60)
        print("MediaPipe 0.10.32+ changed the API structure.")
        print("\nSOLUTION: Install a compatible version with legacy solutions:")
        print("  pip3 uninstall mediapipe")
        print("  pip3 install 'mediapipe<0.10.8'")
        print("\nOr try MediaPipe 0.10.7:")
        print("  pip3 install mediapipe==0.10.7")
        print("="*60)
        cap.release()
        return
    
    # Initialize hand tracker
    try:
        tracker = HandTracker()
        print("MediaPipe hand tracker initialized")
    except Exception as e:
        print(f"\nERROR: Failed to initialize MediaPipe tracker: {e}")
        print("\nMediaPipe 0.10.32+ changed the API. Install a compatible version:")
        print("  pip3 uninstall mediapipe")
        print("  pip3 install 'mediapipe<0.10.8'")
        print("\nOr specifically:")
        print("  pip3 install mediapipe==0.10.7")
        cap.release()
        return
    
    # Server socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
    
    try:
        server.bind(('0.0.0.0', args.port))
    except OSError as e:
        print(f"ERROR: Could not bind to port {args.port}: {e}")
        print(f"Maybe another instance is running? Kill it with: lsof -ti:{args.port} | xargs kill -9")
        return
    
    server.listen(1)
    
    print("\n" + "="*60)
    print(f"MediaPipe Hand Server running on port {args.port}")
    print("Waiting for connection from OrbStack...")
    print("="*60 + "\n")
    print("Controls:")
    print("  - Move your hand in 3D space")
    print("  - X: left/right")
    print("  - Y: up/down")
    print("  - Z: closer/farther (estimated from hand size)")
    print("\nPress Ctrl+C to stop\n")
    
    # Preview window
    show_preview = not args.no_preview
    if show_preview:
        cv2.namedWindow("Hand Tracking (Q to hide)", cv2.WINDOW_NORMAL)
    
    while True:
        try:
            server.settimeout(0.05)  # Non-blocking accept
            try:
                conn, addr = server.accept()
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"\n*** Connected: {addr} ***\n")
                
                frame_count = 0
                start_time = time.time()
                last_hand_data = None
                
                try:
                    while True:
                        ret, frame = cap.read()
                        if not ret:
                            continue
                        
                        # Process hand tracking
                        hand_data = tracker.process_frame(frame)
                        
                        # Send hand data as JSON
                        data = json.dumps(hand_data).encode('utf-8')
                        try:
                            conn.sendall(struct.pack(">I", len(data)) + data)
                        except (BrokenPipeError, ConnectionResetError):
                            print("\nClient disconnected")
                            break
                        
                        frame_count += 1
                        last_hand_data = hand_data
                        
                        # Show stats every 30 frames
                        if frame_count % 30 == 0:
                            elapsed = time.time() - start_time
                            fps = frame_count / elapsed
                            if hand_data['detected']:
                                print(f"FPS: {fps:.1f} | Hand: X={hand_data['x']:+.2f} Y={hand_data['y']:+.2f} Z={hand_data['z']:+.2f}", end='\r')
                            else:
                                print(f"FPS: {fps:.1f} | No hand detected                    ", end='\r')
                        
                        # Show preview
                        if show_preview:
                            # Draw tracking visualization
                            if hand_data['detected']:
                                h, w = frame.shape[:2]
                                cx = int((1 - hand_data['x']) * w / 2)  # Undo the inversion for display
                                cy = int((1 - hand_data['y']) * h / 2)
                                
                                # Draw crosshair at palm position
                                color = (0, 255, 0) if hand_data['detected'] else (0, 0, 255)
                                cv2.circle(frame, (cx, cy), 10, color, 2)
                                cv2.line(frame, (cx - 20, cy), (cx + 20, cy), color, 2)
                                cv2.line(frame, (cx, cy - 20), (cx, cy + 20), color, 2)
                                
                                # Draw depth indicator (size of circle)
                                depth_radius = int(30 + hand_data['z'] * 20)
                                cv2.circle(frame, (cx, cy), max(5, depth_radius), (255, 0, 0), 1)
                                
                                # Text info
                                cv2.putText(frame, f"X:{hand_data['x']:+.2f} Y:{hand_data['y']:+.2f} Z:{hand_data['z']:+.2f}", 
                                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                            else:
                                cv2.putText(frame, "No hand detected", (10, 30), 
                                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                            
                            cv2.imshow("Hand Tracking (Q to hide)", frame)
                            key = cv2.waitKey(1) & 0xFF
                            if key == ord('q'):
                                show_preview = False
                                cv2.destroyAllWindows()
                            elif key == 27:  # ESC
                                raise KeyboardInterrupt
                                
                except (BrokenPipeError, ConnectionResetError):
                    print("\nClient disconnected")
                    continue
                    
            except socket.timeout:
                # No connection yet, just show preview
                ret, frame = cap.read()
                if ret:
                    # Still track hand for preview
                    hand_data = tracker.process_frame(frame)
                    
                    if show_preview:
                        if hand_data['detected']:
                            h, w = frame.shape[:2]
                            cx = int((1 - hand_data['x']) * w / 2)
                            cy = int((1 - hand_data['y']) * h / 2)
                            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), 2)
                            cv2.putText(frame, f"X:{hand_data['x']:+.2f} Y:{hand_data['y']:+.2f} Z:{hand_data['z']:+.2f}", 
                                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                        else:
                            cv2.putText(frame, "Waiting for connection...", (10, 30), 
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 2)
                        
                        cv2.imshow("Hand Tracking (Q to hide)", frame)
                        key = cv2.waitKey(1) & 0xFF
                        if key == ord('q'):
                            show_preview = False
                            cv2.destroyAllWindows()
                        elif key == 27:
                            raise KeyboardInterrupt
                continue
                
        except KeyboardInterrupt:
            print("\n\nShutting down...")
            break
    
    cap.release()
    cv2.destroyAllWindows()
    server.close()
    print("Done")


if __name__ == '__main__':
    main()
