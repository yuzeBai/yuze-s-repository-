#!/usr/bin/env python3
# coding=utf-8
import queue

import rospy
from cv_bridge import CvBridge
import cv2
import smach
import smach_ros
from sensor_msgs.msg import Image
from yolov4 import Detector


class YOLO(smach.State):
    feedback_name_list = []
    feedback_number_list = []
    result_name_list = []
    result_number_list = []

    def __init__(self):

        smach.State.__init__(self, outcomes=['succeeded', 'SEARCH_CAKE'], output_keys=['name_list', 'number_list'])
        self.image_queue = queue.Queue()
        self.result_times_stamp = 0
        self.count = 1
        self.skip = 20
        self.frame = 0
        self.cv_image = None
        self.bridge = CvBridge()
        self.cam_subs = rospy.Subscriber("/camera/image", Image, self.img_callback)
        self.detector = Detector(gpu_id=0, config_path='/opt/darknet/cfg/yolov4.cfg',
                                 weights_path='/opt/darknet/yolov4.weights',
                                 lib_darknet_path='/opt/darknet/libdarknet.so',
                                 meta_path='src/second_coursework/yolo_package/config/coco.data')

    def img_callback(self, msg):
        if self.frame % self.skip == 0:
            self.image_queue.put(self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8'))
        self.frame = (self.frame + 1) % self.skip

    def execute(self, userdata):
        if not self.image_queue.empty():
            img_arr = cv2.resize(self.image_queue.get(),
                                 (self.detector.network_width(), self.detector.network_height()))
            detections = self.detector.perform_detect(image_path_or_buf=img_arr, show_image=True)
            for detection in detections:
                box = detection.left_x, detection.top_y, detection.width, detection.height
                print(f'{detection.class_name.ljust(10)} | {detection.class_confidence * 100:.1f} % | {box}')
                rospy.loginfo(detection.class_name)
                if detection.class_name not in self.feedback_name_list:
                    self.feedback_name_list.append(detection.class_name)
                    self.count = 1
                    self.feedback_number_list.append(self.count)
                elif "cake" in self.feedback_name_list:
                    while (1):
                        i = 1
                    # return 'aborted'
                else:
                    index = self.feedback_name_list.index(detection.class_name)
                    self.feedback_number_list[index] = self.feedback_number_list[index] + 1
            return 'succeeded'
        return 'preempted'


if __name__ == '__main__':
    rospy.init_node('yolo_ros_itr')
    yolo_ros = YOLO()
    rospy.spin()
