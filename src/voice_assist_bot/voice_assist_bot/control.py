import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.subscription = self.create_subscription(String, 'voice_commands', self.command_callback, 10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.energy = 100
        self.get_logger().info("Control Node Initialized.")

    def command_callback(self, msg):
        command = msg.data
        twist = Twist()

        if "forward" in command:
            duration = 2  # default time
            words = command.split()
            if len(words) > 1 and words[1].isdigit():
                duration = int(words[1])
            twist.linear.x = 0.2
            self.move_for_time(twist, duration)
        elif "left" in command:
            twist.angular.z = 1
            self.move_for_time(twist, 2)
        elif "right" in command:
            twist.angular.z = -1
            self.move_for_time(twist, 2)
        elif "heal" in command or "kaboom" in command:
            self.energy = min(100, self.energy + 20)
            self.get_logger().info(f"Energy replenished: {self.energy}")
        else:
            self.get_logger().info(f"Unknown command: {command}")

        self.energy = max(0, self.energy - 5)
        self.get_logger().info(f"Current energy: {self.energy}")

    def move_for_time(self, twist, duration):
        self.publisher.publish(twist)
        self.get_logger().info(f"Moving for {duration} seconds...")
        self.create_timer(duration, lambda: self.publisher.publish(Twist()))  # Stop after time

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
