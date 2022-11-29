#!/usr/bin/env python3

import rospy
import geometry_msgs.msg as geo
from second_coursework.srv import GetRoomCoord, GetRoomCoordResponse, GetRoomCoordRequest

rospy.init_node('roomservice')

room_get_point_proxy = rospy.ServiceProxy('room_get_point', GetRoomCoord)

room_get_point_proxy.wait_for_service()

req = GetRoomCoordRequest()

req.room_name = "B"

resp: GetRoomCoordResponse = room_get_point_proxy(req.room_name)

rospy.loginfo(resp)
