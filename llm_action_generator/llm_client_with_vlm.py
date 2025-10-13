# llm_ros_client_action_style.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import requests

class LLMClientNode(Node):
    def __init__(self):
        super().__init__('llm_client_with_action_prompt')
        self.scene_sub = self.create_subscription(
            String, '/scene_info', self.scene_callback, 10
        )
        self.response_pub = self.create_publisher(String, '/llm_response', 10)

        self.server_url = "http://localhost:5000/generate"
        self.latest_scene = None

        # Example prompt containing target object and optional place location
        self.user_prompt = "Pick up the cup and place it at x=0.6, y=0.17, z=0.10"

        self.timer = self.create_timer(5.0, self.process_prompt)
        self.get_logger().info("LLM ROS client node started with prompt: " + self.user_prompt)

    def scene_callback(self, msg: String):
        try:
            self.latest_scene = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Error parsing scene: {e}")

    def process_prompt(self):
        if not self.latest_scene:
            self.get_logger().warn("No scene received yet.")
            return

        prompt_lower = self.user_prompt.lower()
        target_object = None
        place_location = None

        # Detect target object from prompt
        for obj in self.latest_scene.get("detections", []):
            label = obj.get("label", "").lower()
            if label in prompt_lower:
                target_object = label
                break

        # Extract optional placing coordinates
        if "place it at" in prompt_lower:
            try:
                loc_str = prompt_lower.split("place it at")[1]
                coords = [float(s.strip("xyz=,")) for s in loc_str.replace(",", " ").split() if any(c.isdigit() or c=='.' for c in s)]
                if len(coords) >= 3:
                    place_location = {"x": coords[0], "y": coords[1], "z": coords[2]}
            except Exception as e:
                self.get_logger().warn(f"Could not parse place location: {e}")

        if target_object:
            target = next((o for o in self.latest_scene.get("detections", []) if o.get("label").lower() == target_object), None)
            if target:
                pose = target.get("pose_3d", {}).get("position", {})
                affordances = ', '.join(target.get("affordances", [])) or "none"

                # Create action-oriented prompt for LLM
                prompt_text = (
                    f"Generate step-by-step robot actions for a manipulator:\n"
                    f"Target object: '{target_object}' at pose x={pose.get('x',0):.2f}, "
                    f"y={pose.get('y',0):.2f}, z={pose.get('z',0):.2f}\n"
                    f"Affordances: {affordances}\n"
                )

                if place_location:
                    prompt_text += (
                        f"Place location: x={place_location['x']:.2f}, "
                        f"y={place_location['y']:.2f}, z={place_location['z']:.2f}\n"
                    )
                else:
                    prompt_text += "No place location specified; stay after grasp.\n"

                prompt_text += "Provide actions in a single line step-by-step sequence like: 1)move to object {target_object}' at pose x={pose}. 2) grasp object {target_object}' at pose {pose}, 3)move to place object object {target_object}' at pose {placepose}, 4)release,  5)return to initial home pose if mentioned."

                self.get_logger().info(f"Sending action-style prompt to LLM:\n{prompt_text}")

                try:
                    response = requests.post(self.server_url, json={"prompt": prompt_text})
                    if response.status_code == 200:
                        llm_output = response.json().get("response", "")
                        out_msg = String()
                        out_msg.data = llm_output
                        self.response_pub.publish(out_msg)
                        self.get_logger().info(f"LLM Action Response:\n{llm_output}")
                    else:
                        self.get_logger().error(f"LLM server returned status {response.status_code}")
                except Exception as e:
                    self.get_logger().error(f"Error contacting LLM server: {e}")
        else:
            msg_out = f"Target object not present in the scene as per prompt: {self.user_prompt}"
            out_msg = String()
            out_msg.data = msg_out
            self.response_pub.publish(out_msg)
            self.get_logger().info(msg_out)


def main(args=None):
    rclpy.init(args=args)
    node = LLMClientNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
