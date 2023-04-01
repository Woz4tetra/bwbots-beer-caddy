import rospy
import actionlib

from std_msgs.msg import Bool

from bw_interfaces.msg import SetRobotStateAction, SetRobotStateGoal, SetRobotStateResult


class SetRobotStateCommand:
    def __init__(self) -> None:
        self.are_motors_enabled = False
        self.are_motors_enabled_time = rospy.Time(0)
        self.publish_interval = rospy.Duration(rospy.get_param("~set_robot_state/publish_interval", 0.25))
        self.stale_message_duration = rospy.Duration(rospy.get_param("~set_robot_state/stale_message_duration", 0.05))
        
        self.set_enabled_pub = rospy.Publisher("set_motors_enabled", Bool, queue_size=10)
        self.is_enabled_sub = rospy.Subscriber("are_motors_enabled", Bool, self.are_motors_enabled_callback, queue_size=10)

        self.action_server = actionlib.SimpleActionServer(
            "set_robot_state",
            SetRobotStateAction,
            execute_cb=self.action_callback, 
            auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("set_robot_state is ready")
    
    def set_enable(self, state):
        msg = Bool()
        msg.data = state
        self.set_enabled_pub.publish(msg)

    def are_motors_enabled_callback(self, msg):
        self.are_motors_enabled = msg.data
        self.are_motors_enabled_time = rospy.Time.now()

    def action_callback(self, goal: SetRobotStateGoal):
        rospy.loginfo(f"Setting robot state: {goal}")

        self.are_motors_enabled_time = rospy.Time(0)
        self.set_enable(goal.enabled)
        start_time = rospy.Time.now()
        current_time = rospy.Time.now()

        aborted = False
        while current_time - start_time < goal.timeout:
            current_time = rospy.Time.now()
            if self.action_server.is_preempt_requested():
                self.action_server.set_aborted(result)
                aborted = True
                break
            if goal.enabled == self.are_motors_enabled and current_time - self.are_motors_enabled_time < self.stale_message_duration:
                break
            rospy.sleep(self.publish_interval)
            self.set_enable(goal.enabled)
        
        result = SetRobotStateResult(goal.enabled == self.are_motors_enabled)
        
        if aborted:
            self.action_server.set_aborted(result, "Interrupted while setting robot state")
        else:
            if result.success:
                rospy.loginfo(f"State set successfully: {result}")
            else:
                rospy.loginfo(f"Failed to set state: {result}")
            self.action_server.set_succeeded(result)
