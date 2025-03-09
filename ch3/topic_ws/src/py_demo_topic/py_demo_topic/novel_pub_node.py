import requests
import rclpy
import time
import threading
import espeakng

from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue


class NovelPubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novels_queue_ = Queue()
        self.novel_publisher_ = self.create_publisher(String, 'novel', 10)
        self.timer_ = self.create_timer(5, self.timer_callback)
        
    def timer_callback(self):
        if self.novels_queue_.qsize() > 0:
            msg = String()
            msg.data = self.novels_queue_.get()
            self.novel_publisher_.publish(msg)
            self.get_logger().info(f"published a line of novel: {msg.data}")
            
    def download_novel(self, url):
        response = requests.get(url)
        response.encoding = 'utf-8'
        self.get_logger().info(f"download complete: {url}.")
        
        for line in response.text.splitlines():
            self.novels_queue_.put(line)
            
    
def main():
    rclpy.init()
    node = NovelPubNode('novel_pub')
    node.download_novel('http://127.0.0.1:8000/novel1.txt')
    rclpy.spin(node)
    rclpy.shutdown()
        

