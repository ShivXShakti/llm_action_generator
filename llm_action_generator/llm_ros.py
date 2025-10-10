# ros2_llm_client.py
import rclpy
from rclpy.node import Node
import requests

class LLMClient(Node):
    def __init__(self):
        super().__init__('llm_client')
        # example usage: call server whenever you need text
    def call_llm(self, prompt):
        r = requests.post("http://localhost:5000/generate", json={"prompt": prompt})
        return r.json()

def main(args=None):
    rclpy.init(args=args)
    node = LLMClient()
    print(node.call_llm("Pick the next robot action to pick object A at (x,y)"))
    rclpy.shutdown()

if __name__ == "__main__":
    main()
