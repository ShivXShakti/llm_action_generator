import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests

class LLMClient(Node):
    def __init__(self):
        super().__init__('llm_client')
        self.subscription = self.create_subscription(
            String, '/robot_instruction', self.listener_callback, 10)
        self.publisher = self.create_publisher(String, '/robot_response', 10)
        self.get_logger().info("LLM Client ready, listening on /robot_instruction")

    def listener_callback(self, msg):
        self.get_logger().info(f"Received: {msg.data}")
        try:
            r = requests.post("http://localhost:5000/generate",
                              json={"prompt": msg.data}, timeout=120)
            reply = r.json().get("response", "No reply")
            self.publisher.publish(String(data=reply))
            self.get_logger().info(f"Response: {reply[:100]}")
        except Exception as e:
            self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)
    node = LLMClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
