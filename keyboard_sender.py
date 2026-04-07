import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class KeyboardSender(Node):
    
    def __init__(self):
        super().__init__('keyboard_sender')
        self.publisher_ = self.create_publisher(Int32, 'keyboard_topic', 10)
        self.key_map = {
            'w': 1,  # Move forward
            'a': 2,  # Turn left
            'x': 3,  # Move backward
            'd': 4,  # Turn right
            's': 5,  # Stop
        }
        self.get_logger().info("Enter 'w', 'a', 's', 'd', 'q', 'z', 'x', or 'c' to send numbers")
        self.create_timer(0.1, self.prompt_input)
                    
    def prompt_input(self):
        key = input("Press a key (w, a, s, d, q, z, x, c): ").lower() #.lower() will make all input lowercase
        if key in self.key_map:
            number = self.key_map[key]
            msg = Int32()
            msg.data = number
            self.publisher_.publish(msg)
            self.get_logger().info(f"Published: ")
        else:
            self.get_logger().info("Invalid key. Use 'w', 'a', 's', 'd', 'q', 'z', 'x', or 'c'.")
            
            
def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Keyboard Sender Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
