import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist  # Changed from Int32
import serial
import time
import threading
import queue
import glob
from datetime import datetime

class MD(Node):
    def __init__(self, port='/dev/ttyACM1', baudrate=115200, timeout=0.1):
        super().__init__('mobile_robot_controller')
       
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.lock = threading.Lock()
        self.connect()
        self.block_command = False
        self.command_queue = queue.Queue(maxsize=1)

        self.speed_limit = 3000
        self.safety_timeout = 2.0  # Reduced to 2 seconds for safer physical testing
        self.last_command_time = datetime.now()

        # Subscribe to standard ROS 2 cmd_vel topic
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.running = True
        command_thread = threading.Thread(target=self.process_command_queue)
        command_thread.start()
        
        self.safety_thread = threading.Thread(target=self.safety_monitor)
        self.safety_thread.daemon = True
        self.safety_thread.start()

        self.get_logger().info("Mobile Robot Controller Node Initialized (Listening to cmd_vel)")

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS, timeout=self.timeout
            )
            self.get_logger().info(f"Connected to {self.ser.port}")
            self.send_data(f"^RWD 30000\r")
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")

    def disconnect(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Connection closed")

    def send_data(self, data):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write(data.encode())
            else:
                self.get_logger().error("Serial port not open")

    def vel_control(self, vel_left=0, vel_right=0):
        try:
            vel_left = int(max(-self.speed_limit, min(self.speed_limit, vel_left)))
            vel_right = int(max(-self.speed_limit, min(self.speed_limit, vel_right)))
            command = (vel_left, vel_right)
            if not self.block_command:
                self.send_data(f"!S 1 {vel_left}\r")
                self.send_data(f"!S 2 {vel_right}\r")
                return 0
            else:
                if not self.command_queue.full():
                    self.command_queue.put(command)
                else:
                    self.command_queue.get_nowait()
                    self.command_queue.put(command)
        except Exception as e:
            self.get_logger().error(f"Error in vel_control: {e}")
            self.disconnect()
            return 1

    def process_command_queue(self):
        while self.running:
            if not self.block_command and not self.command_queue.empty():
                vel_left, vel_right = self.command_queue.get()
                self.send_data(f"!S 1 {vel_left}\r")
                self.send_data(f"!S 2 {vel_right}\r")
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
        
        # Differential Drive Kinematics Map
        # linear.x is forward/backward, angular.z is turning left/right
        
        # NOTE: You will need to tune this scaling factor based on your robot's physical size
        # and how the 0-3000 motor command translates to actual meters per second.
        velocity_scaler = 200.0  
        rotation_scaler = 150.0

        vel_left = (msg.linear.x * velocity_scaler) - (msg.angular.z * rotation_scaler)
        vel_right = (msg.linear.x * velocity_scaler) + (msg.angular.z * rotation_scaler)
        
        self.vel_control(vel_left, vel_right)
            
def list_tty_device():
    while True:
        devices = glob.glob('/dev/ttyACM*')
        if devices:
            for device in devices:
                return device  # Fixed loop logic to return directly
        else:
            print("No ttyACM devices found. Waiting for connection...")
            time.sleep(2)
    
def main(args=None):
    rclpy.init(args=args)
    device = list_tty_device()
    node = MD(port=device, baudrate=115200, timeout=0.1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Mobile Robot Controller Node.")
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.running = False
        node.vel_control(0,0) # Force stop on shutdown
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
