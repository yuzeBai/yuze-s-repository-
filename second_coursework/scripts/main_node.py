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
    result = SearchResult()
    feedback = SearchFeedback()

    # status = Searchstatus()
    def __init__(self):
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
                                                response_slots=['point'], ),
                                   transitions={'succeeded': 'cc'},
                                   remapping={'room_name': 'sm_room_name',
                                              'point': 'request_coord'})

            def child_term_cb(outcome_map):
                if outcome_map['GO_TO_ROOM'] == 'succeeded':

                    return True
                return False

            cc = smach.Concurrence(
                outcomes=['succeeded'],
                input_keys=['request_coord', 'feedback_name_list', 'feedback_number_list'],
                default_outcome='succeeded',
                child_termination_cb=child_term_cb, )
            # outcome_map={'succeeded': {'GO_TO_ROOM': 'succeeded',
            #                            'FEEDBACK': 'succeeded',
            #                            },
            #              'succeeded': {'FEEDBACK': 'succeeded'}
            #              })

            with cc:
                def goal_callback(userdata, default_goal):
                    goal = MoveBaseGoal()
                    goal.target_pose.header.stamp = rospy.get_rostime()
                    goal.target_pose.header.frame_id = 'map'
                    goal.target_pose.pose.position.x = userdata.target_pose.x
                    goal.target_pose.pose.position.y = userdata.target_pose.y
                    goal.target_pose.pose.orientation.w = 1
                    return goal

                smach.Concurrence.add('GO_TO_ROOM',
                                      SimpleActionState('move_base', MoveBaseAction,
                                                        input_keys=['target_pose'],
                                                        goal_cb=goal_callback,
                                                        goal_slots=['target_pose']),
                                      remapping={'target_pose': 'request_coord'})

                smach.Concurrence.add('SEARCH_CAKE', YOLO(),
                                      remapping={'name_list': 'feedback_name_list',
                                                 'number_list': 'feedback_number_list'}
                                      )
                sm_sub = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted', 'GO_TO_ROOM'])

                @smach.cb_interface(
                    outcomes=['succeeded', 'preempted'])
                def feedback_cb(ud):
                    self.feedback.current_numbers = YOLO.number_list
                    self.feedback.current_name = YOLO.name_list
                    self.server.publish_feedback(self.feedback)
                    rate.sleep()
                    jump_out()
                    if sm_sub.preempt_requested():
                        sm_sub.service_preempt()
                        return 'preempted'
                    return 'succeeded'

                with sm_sub:
                    def jump_out():
                        return 'preempted'
                    smach.StateMachine.add('feedback_pub', CBState(feedback_cb),
                                           transitions={'succeeded': 'feedback_pub',
                                                        'preempted': 'GO_TO_ROOM',
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
        self.server.set_succeeded(self.result)
        return outcome


if __name__ == '__main__':
    rospy.init_node('main_node')
    server = SearchActionServer()
    rospy.spin()
