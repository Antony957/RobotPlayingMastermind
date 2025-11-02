import os
from openai import OpenAI
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class Client(Node):
    def __init__(self):
        super().__init__('llm_service')
        self.pub = self.create_publisher(String, 'llm_pub', 10)
        self.sub = self.create_subscription(String, 'llm_sub', self.handle, 10)
        self.client = OpenAI(api_key = os.environ.get("API_KEY"), base_url = "https://api.deepseek.com")

    def handle(self, msg):
        self.get_logger().info(msg)
    
def main():
    rclpy.init()
    client = Client()
    try:
        rclpy.spin(client)
    except KeyboardInterrupt:
        pass
    finally:
        client.destory_node()
        rclpy.shutdown()