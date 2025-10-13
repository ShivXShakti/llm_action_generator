#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import requests

class LLMClientNode(Node):
    def __init__(self):
        super().__init__('llm_client_node')

        # Publisher to output LLM response
        self.pub = self.create_publisher(String, 'llm_response', 10)

        # Subscriber to take user prompts
        self.sub = self.create_subscription(
            String,
            'llm_prompt',
            self.prompt_callback,
            10
        )

        self.server_url = 'http://localhost:5000/generate'
        self.get_logger().info("LLM Client Node started")

    def prompt_callback(self, msg):
        prompt = msg.data
        self.get_logger().info(f"Sending prompt to LLM server: {prompt}")

        try:
            # Send POST request to the FastAPI server
            response = requests.post(self.server_url, json={"prompt": prompt})
            if response.status_code == 200:
                data = response.json()
                llm_text = data['response']  # cleaned text from server
                self.get_logger().info(f"LLM Response: {llm_text}")

                # Publish the response
                msg_out = String()
                msg_out.data = llm_text
                self.pub.publish(msg_out)
            else:
                self.get_logger().error(f"Server returned status code {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"Error contacting LLM server: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = LLMClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
