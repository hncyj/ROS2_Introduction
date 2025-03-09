import requests
import rclpy
import time
import threading
import espeakng

from rclpy.node import Node
from example_interfaces.msg import String
from queue import Queue

class NovelSubNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        self.novels_queue_ = Queue()
        self.novel_subscriber_ = self.create_subscription(
            String, 'novel', self.novel_callback, 10
        )
        
        self.speech_thread = threading.Thread(target=self.speack_thread)
        self.speech_thread.start()
        
    def novel_callback(self, msg):
        self.novels_queue_.put(msg)
        
    def speak_thread(self):
        speaker = espeakng.Speaker()
        speaker.voice = 'en'
        
        while rclpy.ok():
            if self.novels_queue_.qsize() > 0:
                text = self.novels_queue_.get()
                self.get_logger().info(f"reading now: {text}")
                speaker.say(text)
                speaker.wait()
            else:
                time.sleep(1)
                
                
def main(args=None):
    rclpy.init(args = args)
    node = NovelSubNode('novel_read')
    rclpy.spin(node)
    rclpy.shutdown()
        
        
        