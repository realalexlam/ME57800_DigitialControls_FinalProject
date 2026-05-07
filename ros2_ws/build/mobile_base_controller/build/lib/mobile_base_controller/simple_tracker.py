import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import csv
import os
from datetime import datetime

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SimpleArucoTracker(Node):
    def __init__(self):
        super().__init__('simple_aruco_tracker')
        
        video_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  
            self.image_callback,
            video_qos)
        
        self.bridge = CvBridge()
        
        # Setup OpenCV ArUco (Legacy API)
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Setup Camera Math (Standard 640x480 lens profile)
        self.camera_matrix = np.array([
            [600.0, 0.0, 320.0],
            [0.0, 600.0, 240.0],
            [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
        half = 0.1 / 2.0
        self.obj_points = np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

        self.last_print_time = 0.0
        self.print_interval = 0.1  # Print to terminal every 100ms so it doesn't lag

        # --- CSV LOGGING SETUP ---
        # Saves the file directly to your workspace folder
        self.csv_file_path = os.path.expanduser('~/ros2_ws/aruco_tracking_data.csv')
        self.csv_file = open(self.csv_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Write the header row
        self.csv_writer.writerow(['Timestamp', 'Marker_ID', 'X_meters', 'Y_meters', 'Z_meters'])
        
        self.get_logger().info(f"Data Logger Started! Saving XYZ data to: {self.csv_file_path}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
            cv_image = cv2.resize(cv_image, (640, 480))
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray_image, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None and len(ids) > 0:
                for i in range(len(ids)):
                    marker_id = ids[i][0]
                    corner_points = corners[i][0]
                    
                    success, rvec, tvec = cv2.solvePnP(
                        self.obj_points, 
                        corner_points, 
                        self.camera_matrix, 
                        self.dist_coeffs
                    )
                    
                    if success:
                        x, y, z = tvec.flatten()
                        
                        # --- WRITE TO CSV ---
                        # Get a clean timestamp for the data point
                        current_timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
                        self.csv_writer.writerow([current_timestamp, marker_id, f"{x:.4f}", f"{y:.4f}", f"{z:.4f}"])
                        
                        # Force save to disk instantly to prevent data loss
                        self.csv_file.flush()
                        
                        # Print to terminal on a timer to avoid lag
                        current_time = time.time()
                        if (current_time - self.last_print_time) >= self.print_interval:
                            self.get_logger().info(f"Logged Marker {marker_id} -> X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}")
                            self.last_print_time = current_time
            
            # (Video window is kept OFF for maximum performance)
            
        except Exception as e:
            self.get_logger().error(f"Image error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleArucoTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down tracker and saving CSV...")
    finally:
        # Safely close the CSV file before shutting down
        node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
