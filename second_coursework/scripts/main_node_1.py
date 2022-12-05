#!/usr/bin/env python3

import rospy
import actionlib
import smach
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from second_coursework.msg import SearchAction, SearchGoal, SearchResult
from second_coursework.srv import GetRoomCoord, GetRoomCoordResponse, GetRoomCoordRequest



class SearchActionServer:
    def __init__(self):
        self.server = actionlib.SimpleActionServer('search', SearchAction, self.execute, False)
        self.server.start()

    def execute(self, goal: SearchGoal):
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted'])
        sm.userdata.sm_room_name = goal.room_name
        with sm:

            smach.StateMachine.add('GET_ROOM_COORD',
                                   ServiceState('room_get_point',
                                                GetRoomCoord,
                                                request_slots=['room_name'],
                                                response_slots=['point'],),
                                   transitions={'succeeded': 'GO_TO_ROOM', 'preempted': 'aborted'},
                                   remapping={'room_name': 'sm_room_name',
                                              'point': 'request_coord'})

            def goal_callback(userdata, default_goal):
                goal = MoveBaseGoal()
                goal.target_pose.header.stamp = rospy.get_rostime()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.pose.position.x = userdata.target_pose.x
                goal.target_pose.pose.position.y = userdata.target_pose.y
                goal.target_pose.pose.orientation.w = 1
                return goal

            smach.StateMachine.add('GO_TO_ROOM',
                                   SimpleActionState('move_base', MoveBaseAction,
                                                     input_keys=['target_pose'],
                                                     goal_cb=goal_callback,
                                                     goal_slots=['target_pose']),
                                   transitions={'succeeded': 'GET_ROOM_COORD', 'preempted': 'GET_ROOM_COORD', 'aborted': 'GET_ROOM_COORD'},
                                   remapping={'target_pose': 'request_coord'})

        intro_server = IntrospectionServer('SearchServer',
                                           sm, '/SM_ROOT')
        intro_server.start()
        outcome = sm.execute()
        intro_server.stop()
        self.server.set_succeeded()
        return outcome


if __name__ == '__main__':
    rospy.init_node('main_node')
    server = SearchActionServer()

    rospy.spin()
