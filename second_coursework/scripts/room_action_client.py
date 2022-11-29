#!/usr/bin/env python3
import rospy
import actionlib
from second_coursework.msg import SearchAction, SearchGoal, SearchResult

rospy.init_node('room_action_client')
client = actionlib.SimpleActionClient('room_action_client', SearchAction)
client.wait_for_server()

goal = SearchGoal()

goal.room_name = 'A'

client.send_goal(goal)
client.wait_for_result()

res: SearchResult = client.get_result()
