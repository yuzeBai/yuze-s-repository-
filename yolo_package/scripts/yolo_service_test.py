#!/usr/bin/env python3

import rospy
import geometry_msgs.msg as geo
from yolo_package.srv import YOLOLastFrame, YOLOLastFrameResponse, YOLOLastFrameRequest

rospy.init_node('yolo_service')

yolo_proxy = rospy.ServiceProxy('/detect_frame', YOLOLastFrame)

yolo_proxy.wait_for_service()

req = YOLOLastFrameRequest()

req.room_name = "B"

resp: GetRoomCoordResponse = room_get_point_proxy(req.room_name)

rospy.loginfo(resp)