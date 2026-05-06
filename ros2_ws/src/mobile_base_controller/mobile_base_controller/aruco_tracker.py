import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

from rclpy.qos import qos_profile_sensor_data
from mobile_base_controller.marker_detector import MarkerDetector

class ArucoTrackerNode(Node):
    def __init__(self):
        super().__init__('aruco_tracker_node')
        
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  
            self.image_callback,
            qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.detector = MarkerDetector(marker_length=0.1)
        
        self.get_logger().info("ArUco Tracker Initialized. Waiting for RealSense images...")

    def image_callback(self, msg):
        try:
            # 1. Grab the color image so we can draw colored boxes on it later
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
            
            # 2. THE FIX: Create a pure, memory-safe Grayscale copy
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # 3. Hand the GRAYSCALE image to the detector to prevent Segfaults
            detections = self.detector.detect(gray_image)
            
            if detections:
                for det in detections:
                    marker_id = det["id"]
                    x, y, z = det["tvec"] 
                    self.get_logger().info(f"Marker {marker_id} Found! XYZ: [X: {x:.3f}, Y: {y:.3f}, Z: {z:.3f}] meters")
                    
                    # 4. Draw the tracking box on the COLOR image
                    cv2.aruco.drawDetectedMarkers(cv_image, [det["corners"]], np.array([marker_id]))
            
            # 5. Show the COLOR video feed
            cv2.imshow("Alchemist Camera Feed", cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down tracker...")
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
