#!/usr/bin/env python3
"""
Camera server to run on your Mac.
Streams camera images to OrbStack over network.

Usage:
    pip3 install opencv-python numpy
    python3 camera_server_for_mac.py

Then in OrbStack, run:
    ros2 launch rbpodo_bringup rbpodo_hand_teleop_bringup.launch.py use_mac_camera:=true enable_hand_teleop:=true ...
"""

import cv2
import socket
import pickle
import struct
import time

def main():
    # Try to open camera
    print("Opening camera...")
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("ERROR: Could not open camera!")
        print("Make sure camera access is allowed in System Settings > Privacy & Security > Camera")
        return
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 30)
    
    # Test capture
    ret, frame = cap.read()
    if not ret:
        print("ERROR: Could not read from camera!")
        return
    
    print(f"Camera opened: {frame.shape[1]}x{frame.shape[0]}")
    
    # Server socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server.bind(('0.0.0.0', 9999))
    except OSError as e:
        print(f"ERROR: Could not bind to port 9999: {e}")
        print("Maybe another instance is running? Kill it with: lsof -ti:9999 | xargs kill -9")
        return
    
    server.listen(1)
    
    print("\n" + "="*50)
    print("Camera server running on port 9999")
    print("Waiting for connection from OrbStack...")
    print("="*50 + "\n")
    print("Press Ctrl+C to stop")
    print("")
    
    # Show preview window on Mac
    cv2.namedWindow("Mac Camera Preview (press Q to hide)", cv2.WINDOW_NORMAL)
    show_preview = True
    
    while True:
        try:
            server.settimeout(0.1)  # Non-blocking accept
            try:
                conn, addr = server.accept()
                print(f"\n*** Connected: {addr} ***\n")
                
                frame_count = 0
                start_time = time.time()
                
                try:
                    while True:
                        ret, frame = cap.read()
                        if not ret:
                            print("Camera read failed")
                            continue
                        
                        # Send frame
                        data = pickle.dumps(frame)
                        size = struct.pack(">L", len(data))
                        conn.sendall(size + data)
                        
                        frame_count += 1
                        
                        # Show FPS every 30 frames
                        if frame_count % 30 == 0:
                            elapsed = time.time() - start_time
                            fps = frame_count / elapsed
                            print(f"Streaming: {fps:.1f} FPS", end='\r')
                        
                        # Show preview
                        if show_preview:
                            cv2.imshow("Mac Camera Preview (press Q to hide)", frame)
                            key = cv2.waitKey(1) & 0xFF
                            if key == ord('q'):
                                show_preview = False
                                cv2.destroyAllWindows()
                        
                except (BrokenPipeError, ConnectionResetError):
                    print("\nClient disconnected")
                    continue
                    
            except socket.timeout:
                # No connection yet, just show preview
                ret, frame = cap.read()
                if ret and show_preview:
                    cv2.imshow("Mac Camera Preview (press Q to hide)", frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        show_preview = False
                        cv2.destroyAllWindows()
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
