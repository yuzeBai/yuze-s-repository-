#!/usr/bin/env python3
import rospy
import sys
import smach
from smach_ros import IntrospectionServer, SimpleActionState, ServiceState
from second_coursework.srv import GetRoomCoord, GetRoomCoordResponse, GetRoomCoordRequest
from move_base_msgs.msg import MoveBaseAction

class WaitForRequest(smach.State):
