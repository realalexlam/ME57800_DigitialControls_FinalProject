import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        
        # --- NEW: RPM Limit ---
        # This is now an absolute target RPM, not a power scale!
        # Adjust this to match the maximum physical RPM of your gearmotors.
        self.speed_limit = 300 
        
        self.block_command = False
        self.command_queue = queue.Queue(maxsize=1)
        self.safety_timeout = 2.0  
        self.last_command_time = datetime.now()

        self.connect()

        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.running = True
        
        command_thread = threading.Thread(target=self.process_command_queue)
        command_thread.start()
        
        self.safety_thread = threading.Thread(target=self.safety_monitor)
        self.safety_thread.daemon = True
        self.safety_thread.start()

        self.read_thread = threading.Thread(target=self.serial_read_loop)
        self.read_thread.daemon = True
        self.read_thread.start()
        
        self.rpm_timer = self.create_timer(0.1, self.request_rpm)

        self.get_logger().info("Motor Driver Initialized (Closed-Loop RPM Mode Active)")

    def connect(self):
        try:
            self.ser = serial.Serial(
                port=self.port, baudrate=self.baudrate,
                parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS, timeout=self.timeout
            )
            self.get_logger().info(f"Connected to {self.ser.port}")
            self.send_data(f"^RWD 30000\r")
            
            # --- NEW: Force Closed-Loop Speed Mode ---
            time.sleep(0.1)
            self.send_data("^MMOD 1 1\r") # Motor 1 to Closed-Loop Speed
            self.send_data("^MMOD 2 1\r") # Motor 2 to Closed-Loop Speed
            self.get_logger().info("Roboteq configured for strict RPM control!")
            
        except serial.SerialException as e:
            self.get_logger().error(f"Error opening serial port: {e}")

    def disconnect(self):
        if self.ser and self.ser.is_open:
            # Revert to open loop (Mode 0) before closing for safety
            self.send_data("^MMOD 1 0\r")
            self.send_data("^MMOD 2 0\r")
            self.ser.close()
            self.get_logger().info("Connection closed")

    def send_data(self, data):
        with self.lock:
            if self.ser and self.ser.is_open:
                self.ser.write(data.encode())
            else:
                self.get_logger().error("Serial port not open")

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
                            clean_str = line.replace('S=', '')
                            vals = clean_str.split(':') 
                            
                            if len(vals) >= 2:
                                try:
                                    left_rpm = -int(vals[0])
                                    right_rpm = -int(vals[1])
                                    self.get_logger().info(f"Live RPM -> Left: {left_rpm} | Right: {right_rpm}")
                                except ValueError:
                                    pass 
                else:
                    time.sleep(0.01)
            except Exception:
                pass

    def vel_control(self, vel_left=0, vel_right=0):
        try:
            # Caps the requested RPM to your safe limit
            vel_left = int(max(-self.speed_limit, min(self.speed_limit, vel_left)))
            vel_right = int(max(-self.speed_limit, min(self.speed_limit, vel_right)))
            command = (vel_left, vel_right)
            
            if not self.block_command:
                # In Closed-Loop Mode, !S commands the exact target RPM!
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
        
        # When you push the joystick to 1.0 (max), it will now request exactly 200 RPM
        velocity_scaler = 200.0  
        rotation_scaler = 150.0

        forward_flip = -1.0 
        turn_flip = 1.0 

        linear_x = msg.linear.x * forward_flip
        angular_z = msg.angular.z * turn_flip

        vel_left = (linear_x * velocity_scaler) - (angular_z * rotation_scaler)
        vel_right = (linear_x * velocity_scaler) + (angular_z * rotation_scaler)
        
        self.vel_control(vel_left, vel_right)
            
def list_tty_device():
    while True:
        devices = glob.glob('/dev/ttyACM*')
        if devices:
            for device in devices:
                return device  
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
        node.vel_control(0,0) 
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()