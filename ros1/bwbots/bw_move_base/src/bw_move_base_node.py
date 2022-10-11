#!/usr/bin/env python3

import rospy
import smach
import smach_ros

import threading

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import RecoveryAction


class BwMoveBase:
    def __init__(self):
        self.name = "bw_move_base"
        rospy.init_node(
            self.name
        )
        # Create SMACH state machine
        self.sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

        # Define userdata
        self.sm.userdata.goal = None
        self.sm.userdata.path = None
        self.sm.userdata.error = None
        self.sm.userdata.clear_costmap_flag = False
        self.sm.userdata.error_status = None

        with self.sm:
            # Monitor topic to get goal from RViz plugin
            smach.StateMachine.add(
                'WAIT_FOR_GOAL',
                smach_ros.MonitorState(
                    '/move_base_simple/goal',
                    PoseStamped,
                    self.goal_cb,
                    output_keys=['goal']
                ),
                transitions={
                    'invalid': 'GET_PATH',
                    'valid': 'WAIT_FOR_GOAL',
                    'preempted': 'preempted'
                }
            )

            # Get path
            smach.StateMachine.add(
                'GET_PATH',
                smach_ros.SimpleActionState(
                    '/move_base_flex/get_path',
                    GetPathAction,
                    goal_slots=['target_pose'],
                    result_slots=['path']
                ),
                transitions={
                    'succeeded': 'EXE_PATH',
                    'aborted': 'STOP_MOTORS',
                    'preempted': 'preempted'
                },
                remapping={
                    'target_pose': 'goal'
                }
            )

            # Execute path
            smach.StateMachine.add(
                'EXE_PATH',
                smach_ros.SimpleActionState(
                    '/move_base_flex/exe_path',
                    ExePathAction,
                    goal_slots=['path']
                ),
                transitions={
                    'succeeded': 'STOP_MOTORS',
                    'aborted': 'RECOVERY',
                    'preempted': 'preempted'
                }
            )

            # Recovery
            smach.StateMachine.add(
                'RECOVERY',
                smach_ros.SimpleActionState(
                    '/move_base_flex/recovery',
                    RecoveryAction,
                    goal_cb=self.recovery_path_goal_cb,
                    input_keys=["error", "clear_costmap_flag"],
                    output_keys=["error_status", 'clear_costmap_flag']
                ),
                transitions={
                    'succeeded': 'GET_PATH',
                    'aborted': 'STOP_MOTORS',
                    'preempted': 'preempted'
                }
            )

            # Stop motors
            smach.StateMachine.add(
                'STOP_MOTORS',
                StopMotorsState(),
                transitions={
                    'finished': 'WAIT_FOR_GOAL',
                }
            )

        rospy.loginfo("%s init done" % self.name)

    def goal_cb(self, userdata, msg):
        # Goal callback for state WAIT_FOR_GOAL
        userdata.goal = msg
        rospy.loginfo("Received goal")
        return False

    def recovery_path_goal_cb(self, userdata, goal):
        # Goal callback for state RECOVERY
        if not userdata.clear_costmap_flag:
            goal.behavior = 'clear_costmap'
            userdata.clear_costmap_flag = True
        else:
            goal.behavior = 'straf_recovery'
            userdata.clear_costmap_flag = False

    def run(self):
        # Execute SMACH plan
        smach_thread = threading.Thread(target=self.sm.execute)
        smach_thread.start()

        rospy.spin()

        # Request the container to preempt
        self.sm.request_preempt()

        # Block until everything is preempted 
        smach_thread.join()


class StopMotorsState(smach.State):
    def __init__(self):
        super(StopMotorsState, self).__init__(
            outcomes=["finished"],
        )
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=100)
    
    def execute(self, userdata):
        self.stop_motors()
        rospy.loginfo("Stopping motors")
        return "finished"

    def stop_motors(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)


if __name__ == "__main__":
    node = BwMoveBase()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.name)
