import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import serial
import time
import threading
import queue
import glob
import csv
import os
from datetime import datetime

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class AlchemistCore(Node):
    def __init__(self, port='/dev/ttyACM1', baudrate=115200, timeout=0.1):
        super().__init__('alchemist_core_node')
       
        # --- 1. MOTOR CONTROLLER SETUP ---
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.lock = threading.Lock()
        
        self.speed_limit = 300 
        self.block_command = False
        self.command_queue = queue.Queue(maxsize=1)
        self.safety_timeout = 2.0  
        self.last_command_time = datetime.now()
        
        # Cache for Live Telemetry
        self.live_left_rpm = 0
        self.live_right_rpm = 0
        
        # Cache for Live Vision
        self.live_marker_id = "Lost"
        self.live_x = 0.0
        self.live_y = 0.0
        self.live_z = 0.0

        self.connect_serial()

        # --- 2. CAMERA & ARUCO SETUP ---
        video_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )
        self.subscription_cam = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, video_qos)
        
        self.subscription_vel = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10)
            
        self.bridge = CvBridge()
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        self.camera_matrix = np.array([
            [600.0, 0.0, 320.0], [0.0, 600.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        
        half = 0.1 / 2.0
        self.obj_points = np.array([
            [-half,  half, 0], [ half,  half, 0],
            [ half, -half, 0], [-half, -half, 0]
        ], dtype=np.float32)

        # --- 3. CSV LOGGING SETUP ---
        self.csv_file_path = os.path.expanduser('~/ros2_ws/alchemist_telemetry.csv')
        self.csv_file = open(self.csv_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['Timestamp', 'Marker_ID', 'X_meters', 'Y_meters', 'Z_meters', 'Left_RPM', 'Right_RPM'])

        self.running = True
        
        # --- 4. START THREADS & HIGH-SPEED TIMERS ---
        threading.Thread(target=self.process_command_queue, daemon=True).start()
        threading.Thread(target=self.safety_monitor, daemon=True).start()
        threading.Thread(target=self.serial_read_loop, daemon=True).start()
        
        # THE SPEED FIX: Ask motors for RPM 50 times a second (0.02s) instead of 10 times (0.1s)
        self.create_timer(0.02, self.request_rpm)
        
        # THE SPEED FIX: Dedicated clock to dump data to the CSV 50 times a second
        self.create_timer(0.02, self.log_to_csv)

        self.get_logger().info(f"Alchemist Core Initialized! High-Speed Logging (50Hz) to: {self.csv_file_path}")

    def connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS, timeout=self.timeout
            )
            self.send_data(f"^RWD 30000\r")
            time.sleep(0.1)
            self.send_data("^MMOD 1 1\r") 
            self.send_data("^MMOD 2 1\r") 
        except serial.SerialException as e:
            self.get_logger().error(f"Serial Error: {e}")

    def disconnect_serial(self):
        if self.ser and self.ser.is_open:
            self.send_data("^MMOD 1 0\r")
            self.send_data("^MMOD 2 0\r")
            self.ser.close()

    def send_data(self, data):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write(data.encode())

    def request_rpm(self):
        if not self.block_command:
            self.send_data("?S\r")

    def serial_read_loop(self):
        while self.running:
            try:
                if self.ser and self.ser.is_open and self.ser.in_waiting > 0:
                    raw_data = self.ser.read(self.ser.in_waiting).decode('utf-8', errors='ignore')
                    lines = raw_data.split('\r')
                    for line in lines:
                        if line.startswith('S='):
                            vals = line.replace('S=', '').split(':') 
                            if len(vals) >= 2:
                                try:
                                    self.live_right_rpm = -int(vals[0])
                                    self.live_left_rpm = -int(vals[1])
                                except ValueError:
                                    pass 
                else:
                    time.sleep(0.01)
            except Exception:
                pass

    def vel_control(self, vel_left=0, vel_right=0):
        try:
            vel_left = int(max(-self.speed_limit, min(self.speed_limit, vel_left)))
            vel_right = int(max(-self.speed_limit, min(self.speed_limit, vel_right)))
            
            if not self.block_command:
                self.send_data(f"!S 1 {vel_right}\r")
                self.send_data(f"!S 2 {vel_left}\r")
                return 0
            else:
                if not self.command_queue.full():
                    self.command_queue.put((vel_left, vel_right))
                else:
                    self.command_queue.get_nowait()
                    self.command_queue.put((vel_left, vel_right))
        except Exception:
            self.disconnect_serial()
            return 1

    def process_command_queue(self):
        while self.running:
            if not self.block_command and not self.command_queue.empty():
                vel_left, vel_right = self.command_queue.get()
                self.send_data(f"!S 1 {vel_right}\r")
                self.send_data(f"!S 2 {vel_left}\r")
            time.sleep(0.1)
            
    def safety_monitor(self):
        while self.running:
            time_since_last_command = datetime.now() - self.last_command_time
            if time_since_last_command.total_seconds() > self.safety_timeout:
                self.block_command = True
                self.vel_control(0, 0)
                self.block_command = False
            time.sleep(0.1)

    def cmd_vel_callback(self, msg):
        self.last_command_time = datetime.now()
        velocity_scaler = 200.0  
        rotation_scaler = 150.0

        linear_x = msg.linear.x * -1.0 
        angular_z = msg.angular.z * 1.0 

        vel_left = (linear_x * velocity_scaler) - (angular_z * rotation_scaler)
        vel_right = (linear_x * velocity_scaler) + (angular_z * rotation_scaler)
        
        self.vel_control(vel_left, vel_right)

    # ==========================================
    #     HIGH-SPEED DECOUPLED FUNCTIONS
    # ==========================================
    def log_to_csv(self):
        # Grabs the freshest cache data exactly 50 times a second
        current_timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        self.csv_writer.writerow([
            current_timestamp, 
            self.live_marker_id, 
            f"{self.live_x:.4f}", f"{self.live_y:.4f}", f"{self.live_z:.4f}", 
            self.live_left_rpm, self.live_right_rpm
        ])
        self.csv_file.flush()

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8").copy()
            cv_image = cv2.resize(cv_image, (640, 480))
            gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            corners, ids, _ = cv2.aruco.detectMarkers(gray_image, self.aruco_dict, parameters=self.aruco_params)
            
            if ids is not None and len(ids) > 0:
                # Update cache with the first marker seen in the frame
                marker_id = ids[0][0]
                success, rvec, tvec = cv2.solvePnP(self.obj_points, corners[0][0], self.camera_matrix, self.dist_coeffs)
                
                if success:
                    x, y, z = tvec.flatten()
                    self.live_marker_id = marker_id
                    self.live_x = x
                    self.live_y = y
                    self.live_z = z
            else:
                self.live_marker_id = "Lost"
                        
        except Exception:
            pass 

def list_tty_device():
    while True:
        devices = glob.glob('/dev/ttyACM*')
        if devices: return devices[0]
        time.sleep(2)
    
def main(args=None):
    rclpy.init(args=args)
    device = list_tty_device()
    node = AlchemistCore(port=device)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down and saving CSV...")
    finally:
        node.running = False
        node.vel_control(0,0) 
        node.csv_file.close()
        node.disconnect_serial()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()