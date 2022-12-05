#!/usr/bin/env python3
# coding=utf-8

import rospy
from cv_bridge import CvBridge
import cv2
import smach
import smach_ros
from sensor_msgs.msg import Image
from yolov4 import Detector


class YOLO(smach.State):
    name_list = []
    number_list = []

    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded'], output_keys=['name_list', 'number_list'])
        self.count = 1
        self.skip = 20  # We will process 1 of every self.skip frames
        self.frame_count = 0
        self.cv_image = None
        self.bridge = CvBridge()
        self.cam_subs = rospy.Subscriber("/camera/image", Image, self.img_callback)
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='src/yolo_package/config/coco.data')

    def img_callback(self, msg):
        self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def execute(self, userdata):
        if len(self.cv_image) > 0:
            img_arr = cv2.resize(self.cv_image, (self.detector.network_width(), self.detector.network_height()))
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
            for detection in detections:
                box = detection.left_x, detection.top_y, detection.width, detection.height
                print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {box}')
                rospy.loginfo(detection.class_name)
                if detection.class_name not in self.name_list:
                    self.name_list.append(detection.class_name)
                    self.count = 1
                    self.number_list.append(self.count)
                else:
                    index = self.name_list.index(detection.class_name)
                    self.number_list[index] = self.number_list[index] + 1
                if detection.class_name == "cake":
                    self.result.objects_name = self.name_list
                    self.result.objects_numbers = self.number_list

            return 'succeeded'


if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLO()
    rospy.spin()
