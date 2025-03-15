import cv2

import sys
sys.path.append('/home/chenyinjie/miniconda3/envs/ros/lib/python3.10/site-packages')

import face_recognition
from ament_index_python.packages import get_package_share_directory

def main():
    default_img_path = get_package_share_directory('py_demo_service') + '/resource/faces.jpg'
    img = cv2.imread(default_img_path)
    face_location = face_recognition.face_locations(
        img, number_of_times_to_upsample=1, model='hog'
    )
    for top, right, bottom, left in face_location:
        cv2.rectangle(img, (left, top), (right, bottom), (255, 0, 0), 4)
    save_img_path = get_package_share_directory('py_demo_service') + '/resource/faces_detected.jpg'
    # 保存图像
    cv2.imwrite(save_img_path, img)
    print(f"img save to: {save_img_path}")