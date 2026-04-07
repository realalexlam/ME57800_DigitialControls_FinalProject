import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import serial
import time
import threading
import queue
import glob
from datetime import datetime

class MD(Node):
    def __init__(self, port='/dev/ttyACM1', baudrate=115200, timeout=0.1):
        super().__init__('mobile_robot_controller')
       
        # Initialize serial connection
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        self.lock = threading.Lock()
        self.connect()
        self.block_command = False
        self.command_queue = queue.Queue(maxsize=1)

        # Motor configuration
        self.turning_speed = 100
        self.default_speed = 100
        self.speed_limit = 3000
        
        # Safety timeout - robot will stop if no command is received in this time
        self.safety_timeout = 20.0  # 20 second
        self.last_command_time = datetime.now()

        # ROS 2 Subscription
        self.subscription = self.create_subscription(Int32, 'keyboard_topic', self.listener_callback, 10)

        # Start monitoring thread
        self.running = True
        #monitoring_thread = threading.Thread(target=self.monitoring_func)
        command_thread = threading.Thread(target=self.process_command_queue)
        #monitoring_thread.start()
        command_thread.start()
        
        # Start safety monitor thread
        self.safety_thread = threading.Thread(target=self.safety_monitor)
        self.safety_thread.daemon = True
        self.safety_thread.start()

        self.get_logger().info("Mobile Robot Controller Node Initialized")
        self.get_logger().info(f"Safety timeout set to {self.safety_timeout} seconds")

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
            vel_left = max(-self.speed_limit, min(self.speed_limit, vel_left))
            vel_right = max(-self.speed_limit, min(self.speed_limit, vel_right))
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
        """Monitor for command timeout and stop robot if needed"""
        while self.running:
            time_since_last_command = datetime.now() - self.last_command_time
            
            # If it's been too long since the last command, stop the robot
            if time_since_last_command.total_seconds() > self.safety_timeout:
                # Only stop if we're not already stopped
                self.block_command = True  # Prevent other commands while we're stopping
                self.vel_control(0, 0)
                self.block_command = False
                self.get_logger().info("Safety timeout reached - stopping robot")
                
            time.sleep(0.1)  # Check frequently

    # Listener callback for ROS 2
    def listener_callback(self, msg):
        command = msg.data
        # update the last command time
        self.last_command_time = datetime.now()
        
        if command == 1:  # 'w' key: move forward
            self.vel_control(self.default_speed, self.default_speed)
            self.get_logger().info("Command received: Move Forward")
        elif command == 2:  # 'a' key: turn left
            self.vel_control(-self.turning_speed, self.turning_speed)
            self.get_logger().info("Command received: Turn Left")
        elif command == 3:  # 'x' key: move backward
            self.vel_control(-self.default_speed, -self.default_speed)
            self.get_logger().info("Command received: Move Backward")
        elif command == 4:  # 'd' key: turn right
            self.vel_control(self.turning_speed, -self.turning_speed)
            self.get_logger().info("Command received: Turn Right")
        elif command == 5:  # 's' key: stop
            self.vel_control(0, 0)
            self.get_logger().info("Command received: Stop")
        else:
            self.get_logger().info("Unknown command received")
            
def list_tty_device():
    while True:
        devices = glob.glob('/dev/ttyACM*')
        if devices:
            for device in devices:
                break
            break  # Exit the loop once devices are found
        else:
            print("No ttyACM devices found. Waiting for connection...")
            time.sleep(2)  # Wait for 2 seconds before checking again
    return device
    
def main(args=None):
    rclpy.init(args=args)
    device = list_tty_device()
    node = MD(port=device, baudrate=115200, timeout=0.1)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Mobile Robot Controller Node.")
    except serial.SerialException as e:
        node.get_logger().info("Serial issue occurred, reconnect again")
        device = list_tty_device()
        node = MD(port=device, baudrate=115200, timeout=0.1)
    except Exception as e:
        node.get_logger().error(f"Error during rclpy.spin: {e}")
    finally:
        node.running = False
        node.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    
