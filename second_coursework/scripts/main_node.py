#!/usr/bin/env python3

import rospy
import actionlib
import smach
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction
from smach import Concurrence, CBState
from smach_ros import ServiceState, SimpleActionState, IntrospectionServer
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from second_coursework.msg import SearchAction, SearchGoal, SearchResult, SearchFeedback
from second_coursework.srv import GetRoomCoord, GetRoomCoordResponse, GetRoomCoordRequest

from yolo_ros import YOLO


class SearchActionServer:

    def __init__(self):
        self.result = SearchResult()
        self.feedback = SearchFeedback()
        self.server = actionlib.SimpleActionServer('search', SearchAction, self.execute, False)
        self.server.start()

    def create_sm(self, goal: SearchGoal):
        rate = rospy.Rate(5)
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        sm.userdata.sm_room_name = goal.room_name
        sm.userdata.feedback_name_list = self.feedback.current_name
        sm.userdata.feedback_number_list = self.feedback.current_numbers

        with sm:
            smach.StateMachine.add('GET_ROOM_COORD',
                                   ServiceState('room_get_point',
                                                GetRoomCoord,
                                                request_slots=['room_name'],
                                                response_slots=['point']),
                                   transitions={'succeeded': 'cc'},
                                   remapping={'room_name': 'sm_room_name',
                                              'point': 'request_coord'})

            def child_term_cb(outcome_map):
                if outcome_map['GO_TO_ROOM'] == 'succeeded':
                    return True
                if outcome_map['SEARCH_CAKE'] == 'succeeded':
                    return True
                return False

            cc = smach.Concurrence(
                outcomes=['succeeded'],
                input_keys=['request_coord', 'feedback_name_list', 'feedback_number_list'],
                default_outcome='succeeded',
                child_termination_cb=child_term_cb, )

            # outcome_map={'succeeded': {'GO_TO_ROOM': 'succeeded'}
            #              #                            'FEEDBACK': 'succeeded',
            #              #                        # 'SEARCH_CAKE': 'succeeded'
            #              #                        },
            #              #          # 'succeeded': {'SEARCH_CAKE': 'succeeded'},
            #              #              'succeeded': {'FEEDBACK': 'succeeded'}
            #              })
            with cc:
                smach.Concurrence.add('SEARCH_CAKE', YOLO(),
                                      remapping={'name_list': 'feedback_name_list',
                                                 'number_list': 'feedback_number_list'}
                                      )

                def goal_callback(userdata, default_goal):
                    goal = MoveBaseGoal()
                    goal_new = SearchGoal()
                    goal.target_pose.header.stamp = rospy.get_rostime()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.pose.position.x = userdata.target_pose.x
                    goal.target_pose.pose.position.y = userdata.target_pose.y
                    goal.target_pose.pose.orientation.w = 1
                    if 'cake' in YOLO.feedback_name_list:
                        while (1):
                            rospy.loginfo("THE CAKE FOUND!")
                            rospy.sleep(40)
                            i = 1
                    return goal

                smach.Concurrence.add('GO_TO_ROOM',
                                      SimpleActionState('move_base', MoveBaseAction,
                                                        input_keys=['target_pose'],
                                                        goal_cb=goal_callback,
                                                        goal_slots=['target_pose']),
                                      remapping={'target_pose': 'request_coord'})

                sm_sub = smach.StateMachine(
                    outcomes=['succeeded', 'aborted', 'preempted', 'GO_TO_ROOM', 'SEARCH_CAKE'])

                @smach.cb_interface(
                    outcomes=['succeeded', 'preempted', 'aborted'])
                def feedback_cb(ud):
                    if "cake" not in YOLO.feedback_name_list:
                        self.feedback.current_numbers = YOLO.feedback_number_list
                        self.feedback.current_name = YOLO.feedback_name_list
                        self.server.publish_feedback(self.feedback)
                        rate.sleep()
                    else:
                        rospy.loginfo("FOUND THE CAKE!")
                        self.result.timestamp = rospy.Time.now()
                        self.result.objects_numbers = YOLO.feedback_number_list
                        self.result.objects_name = YOLO.feedback_name_list
                        self.server.set_succeeded(self.result, 'FOUND THE CAKE')
                        return 'aborted'
                    jump_out()
                    if sm_sub.preempt_requested():
                        sm_sub.service_preempt()
                        return 'preempted'
                    return 'succeeded'

                with sm_sub:
                    def jump_out():
                        return 'preempted'

                    smach.StateMachine.add('FEEDBACK_PUB', CBState(feedback_cb),
                                           transitions={'succeeded': 'FEEDBACK_PUB',
                                                        'preempted': 'GO_TO_ROOM',
                                                        'aborted': 'aborted'
                                                        }
                                           )
                smach.Concurrence.add('FEEDBACK', sm_sub)
            smach.StateMachine.add('cc', cc, transitions={'succeeded': 'GET_ROOM_COORD'})
        return sm

    def execute(self, goal: SearchGoal):
        sm = self.create_sm(goal)
        intro_server = IntrospectionServer('SearchServer',
                                           sm, '/SM_ROOT')
        intro_server.start()
        outcome = sm.execute()
        intro_server.stop()
        return outcome


if __name__ == '__main__':
    rospy.init_node('main_node')
    server = SearchActionServer()
    rospy.spin()
