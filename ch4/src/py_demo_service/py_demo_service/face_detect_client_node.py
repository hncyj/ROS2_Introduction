import rclpy
import cv2
import sys
sys.path.append('/home/chenyinjie/miniconda3/envs/ros/lib/python3.10/site-packages')
from rclpy.node import Node
from ch4_interfaces.srv import FaceDetector
from sensor_msgs.msg import Image
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge

class FaceDetectClient(Node):
    def __init__(self):
        super().__init__('face_detect_client')
        self.client = self.create_client(FaceDetector, '/face_detect')
        self.bridge = CvBridge()
        self.img1_path =  get_package_share_directory('py_demo_service') + '/resource/test1.jpeg'
        self.image = cv2.imread(self.img1_path)
        
    def send_request(self):
        while self.client.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f'wait for server online.....')
        
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        self.get_logger().info(f"results recived, total face number: {response.number}, cost time: {response.use_time}.")
        self.show_face_locations(response)
        
    
    def show_face_locations(self, response):
        for i in range(response.number):
            top = response.top[i]
            right = response.right[i]
            bottom = response.bottom[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left, top), (right, bottom), (255, 0, 0), 2)
        
        cv2.imshow('Face detection results', self.image)
        cv2.waitKey(0)
        
        
def main(args=None):
    rclpy.init(args=args)
    face_detect_client = FaceDetectClient()
    face_detect_client.send_request()
    rclpy.shutdown()