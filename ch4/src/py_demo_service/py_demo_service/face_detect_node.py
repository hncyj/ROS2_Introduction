import rclpy
import cv2
import time
import sys
sys.path.append('/home/chenyinjie/miniconda3/envs/ros/lib/python3.10/site-packages')
import face_recognition
from rclpy.node import Node
from ch4_interfaces.srv import FaceDetector
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge # 格式转换



class FaceDetectNode(Node):
    def __init__(self):
        super().__init__(node_name='face_detect_node')
        self.bridge = CvBridge()
        self.service = self.create_service(
            FaceDetector, 'face_detect', self.detect_face_callback
        ) # params: interface, service name, request handle callback func
        self.default_img_path = get_package_share_directory('py_demo_service') + '/resource/faces.jpg'
        self.upsample = 1
        self.model = "hog"
        
    def detect_face_callback(self, request, response):
        if request.image.data:
            cv_img = self.bridge.imgmsg_to_cv2(request.image)
        else:
            cv_img = cv2.imread(self.default_img_path)
        
        start_time = time.time()
        self.get_logger().info('load img... success!    start detecting...\n')
        face_locations = face_recognition.face_locations(
            cv_img, number_of_times_to_upsample=self.upsample, model=self.model
        )
        end_time = time.time()
        self.get_logger().info(f'detect finished, time cost: {end_time - start_time}')
        
        response.number = len(face_locations)
        response.use_time = end_time - start_time
        
        for top, right, bottom, left in face_locations:
            response.top.append(top)
            response.right.append(right)
            response.bottom.append(bottom)
            response.left.append(left)
            
        return response

        
def main(args=None):
    rclpy.init(args=args)
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()