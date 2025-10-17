import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import String
import requests
import json
import re

class LLAMARobotExecutor(Node):
    def __init__(self):
        super().__init__('llama_robot_executor')

        # Publishers for robot control
        self.pose_pub = self.create_publisher(Pose, '/robot/target_pose', 10)
        self.cmd_pub = self.create_publisher(String, '/robot/command', 10)

        # Example scene parameters (can be dynamic)
        self.cup_pos = (0.4, 0.2, 0.3)
        self.bottle_pos = (0.3, -0.3, 0.4)
        self.hover_height = 0.06

        # REST API URL
        self.LLAMA_URL = "http://0.0.0.0:5000/generate"

        # Execute plan
        plan = self.get_plan_from_llama()
        if plan:
            self.execute_plan(plan)

    def get_plan_from_llama(self):
        # Construct f-string prompt with hover positions
        prompt = f"""
You are a robot task planner. 
Current scene:
- Red cup at {self.cup_pos}
- Blue bottle at {self.bottle_pos}

Command: "Move the red cup next to the blue bottle."

Generate a structured step-by-step action plan for a 7-DOF robot manipulator. 
Return JSON only with key 'plan', each step like:
- move_to(x, y, z)
- close_gripper()
- open_gripper()

Include all intermediate steps required to safely pick up the red cup and place it next to the blue bottle. 
Use a hover height of {self.hover_height} meters above the objects. 
For example, if cup z is {self.cup_pos[2]}, first move to z={self.cup_pos[2]+self.hover_height}, then lower to z={self.cup_pos[2]}.
Similarly for the bottle.

Example output format:

{{
  "plan": [
    "move_to(<cup_x>, <cup_y>, <cup_z + hover_height>)",
    "move_to(<cup_x>, <cup_y>, <cup_z>)",
    "close_gripper()",
    "move_to(<cup_x>, <cup_y>, <cup_z + hover_height>)",
    "move_to(<bottle_x>, <bottle_y>, <bottle_z + hover_height>)",
    "move_to(<bottle_x>, <bottle_y>, <bottle_z>)",
    "open_gripper()",
    "move_to(<bottle_x>, <bottle_y>, <bottle_z + hover_height>)"
  ]
}}
"""
        try:
            payload = {"prompt": prompt, "max_new_tokens": 300, "temperature": 0.1}
            response = requests.post(self.LLAMA_URL, json=payload).json()
            raw_text = response.get("response", "")

            # Extract JSON from the response
            match = re.search(r"\{(?:.|\n)*\}", raw_text)
            if match:
                plan_json = json.loads(match.group())
                self.get_logger().info("Received plan from LLaMA.")
                return plan_json["plan"]
            else:
                self.get_logger().error("No JSON plan found in LLaMA response.")
                return None
        except Exception as e:
            self.get_logger().error(f"Failed to get plan from LLaMA: {e}")
            return None

    def execute_plan(self, plan):
        self.get_logger().info("Executing plan...")
        for step in plan:
            step = step.strip()
            if step.startswith("move_to"):
                # Parse coordinates from string
                coords = step[8:-1].split(",")  # remove 'move_to(' and ')'
                try:
                    x, y, z = map(float, coords)
                    pose_msg = Pose()
                    pose_msg.position.x = x
                    pose_msg.position.y = y
                    pose_msg.position.z = z
                    self.pose_pub.publish(pose_msg)
                    self.get_logger().info(f"Moving to: x={x}, y={y}, z={z}")
                except:
                    self.get_logger().warn(f"Invalid move_to coordinates: {step}")
            else:
                # Publish command string (gripper actions)
                cmd_msg = String()
                cmd_msg.data = step
                self.cmd_pub.publish(cmd_msg)
                self.get_logger().info(f"Executing command: {step}")

def main(args=None):
    rclpy.init(args=args)
    node = LLAMARobotExecutor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
