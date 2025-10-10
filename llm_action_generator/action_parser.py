import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re

class ActionParser(Node):
    def __init__(self):
        super().__init__('action_parser')
        self.subscription = self.create_subscription(
            String, '/robot_response', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, '/arm_command', 10)
        self.get_logger().info("Action Parser ready, listening on /robot_response")

    def listener_callback(self, msg):
        text = msg.data.lower()
        self.get_logger().info(f"Received LLM output: {text}")

        # Default parsed values
        arm = None
        action = None
        object_name = None

        # Identify which arm
        if "left arm" in text:
            arm = "LEFT_ARM"
        elif "right arm" in text:
            arm = "RIGHT_ARM"

        # Identify action keywords
        if "pick" in text or "grasp" in text:
            action = "PICK"
        elif "place" in text or "put" in text:
            action = "PLACE"
        elif "move" in text:
            action = "MOVE"

        # Try to extract object name (simplified)
        match = re.search(r"(cup|bottle|can|box|object|cube)", text)
        if match:
            object_name = match.group(1).upper()

        if arm and action:
            cmd = f"{arm} {action}"
            if object_name:
                cmd += f" {object_name}"
            else:
                cmd += " UNKNOWN_OBJECT"

            self.publisher.publish(String(data=cmd))
            self.get_logger().info(f"Published command: {cmd}")
        else:
            self.get_logger().warn("Could not parse action properly.")

def main(args=None):
    rclpy.init(args=args)
    node = ActionParser()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
