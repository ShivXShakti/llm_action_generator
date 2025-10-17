import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import re

class LLMActionExtractor(Node):
    def __init__(self):
        super().__init__('llm_action_extractor')
        self.subscription = self.create_subscription(
            String,
            '/llm_response',
            self.response_callback,
            10
        )
        self.get_logger().info("LLM Action Extractor started. Waiting for /llm_response messages...")

    def response_callback(self, msg: String):
        raw_text = msg.data.strip()
        self.get_logger().info("\n===== Raw LLM Response =====\n" + raw_text)

        # Normalize text (lowercase for easier matching)
        text = raw_text.lower()

        # Define patterns to extract main actions
        actions = {
            "move": re.findall(r"move\s+to\s+position.*?(?=\n|\.|$)", text),
            "close_gripper": re.findall(r"close\s+gripper.*?(?=\n|\.|$)", text),
            "open_gripper": re.findall(r"open\s+gripper.*?(?=\n|\.|$)", text),
            "return_home": re.findall(r"return\s+to\s+home.*?(?=\n|\.|$)", text)
        }

        extracted_actions = []
        for action, matches in actions.items():
            for match in matches:
                extracted_actions.append(action)

        if extracted_actions:
            self.get_logger().info("\n===== Extracted Actions =====")
            for i, action in enumerate(extracted_actions, 1):
                self.get_logger().info(f"{i}. {action}")
            self.get_logger().info("=============================\n")
        else:
            self.get_logger().warn("No standard actions detected in LLM response.")


def main(args=None):
    rclpy.init(args=args)
    node = LLMActionExtractor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
