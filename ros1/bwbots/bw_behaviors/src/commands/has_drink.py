import rospy
import actionlib
from bw_interfaces.msg import LoadCell
from bw_interfaces.msg import (
    HasDrinkAction,
    HasDrinkGoal,
    HasDrinkFeedback,
    HasDrinkResult,
)
from bw_interfaces.msg import (
    WaitForDrinkAction,
    WaitForDrinkGoal,
    WaitForDrinkFeedback,
    WaitForDrinkResult,
)


class HasDrinkCommand:
    def __init__(self) -> None:
        self.load_cell_sub = rospy.Subscriber(
            "/bw/load_cell/mass", LoadCell, self.load_cell_callback, queue_size=10
        )
        self.prev_reading = rospy.Time(0)
        self.mass = 0.0

        self.has_drink_action_server = actionlib.SimpleActionServer(
            "has_drink",
            HasDrinkAction,
            execute_cb=self.has_drink_action_callback,
            auto_start=False,
        )
        self.has_drink_action_server.start()

        self.wait_for_drink_action_server = actionlib.SimpleActionServer(
            "wait_for_drink",
            WaitForDrinkAction,
            execute_cb=self.wait_for_drink_action_callback,
            auto_start=False,
        )
        self.wait_for_drink_action_server.start()
        rospy.loginfo("has_drink is ready")

    def load_cell_callback(self, msg):
        self.mass = msg.mass
        self.prev_reading = rospy.Time.now()

    def has_drink_action_callback(self, goal: HasDrinkGoal):
        timeout = goal.timeout
        mass_threshold = goal.mass_threshold
        result = HasDrinkResult()
        result.success = False

        aborted = False
        start_time = rospy.Time.now()
        current_time = rospy.Time.now()
        while current_time - start_time < timeout:
            current_time = rospy.Time.now()
            if self.has_drink_action_server.is_preempt_requested():
                aborted = True
                break
            if self.prev_reading > start_time:
                result.success = True
                result.has_drink = self.mass > mass_threshold
                break
            rospy.sleep(0.1)

        if aborted:
            self.has_drink_action_server.set_aborted(
                result, "Interrupted while checking for a drink"
            )
        else:
            if result.success:
                if result.has_drink:
                    rospy.loginfo(
                        f"Robot is carrying a drink. Mass reading is {self.mass}"
                    )
                else:
                    rospy.loginfo(
                        f"Robot is not carrying a drink. Mass reading is {self.mass}"
                    )
            else:
                rospy.loginfo(f"Failed to get drink status")
            self.has_drink_action_server.set_succeeded(result)

    def wait_for_drink_action_callback(self, goal: WaitForDrinkGoal):
        timeout = goal.timeout
        mass_threshold = goal.mass_threshold
        expected_state = goal.expected_state
        result = WaitForDrinkResult()
        result.success = False

        has_drink = False
        aborted = False
        start_time = rospy.Time.now()
        current_time = rospy.Time.now()
        while current_time - start_time < timeout:
            current_time = rospy.Time.now()
            if self.wait_for_drink_action_server.is_preempt_requested():
                aborted = True
                break
            has_drink = self.mass > mass_threshold
            feedback = WaitForDrinkFeedback(has_drink)
            self.wait_for_drink_action_server.publish_feedback(feedback)
            if self.prev_reading > start_time:
                result.success = True
                if has_drink == expected_state:
                    result.state_matched = has_drink == expected_state
                    break
            rospy.sleep(0.1)

        if aborted:
            self.wait_for_drink_action_server.set_aborted(
                result, "Interrupted while checking for a drink"
            )
        else:
            if result.success:
                if result.state_matched:
                    rospy.loginfo(
                        f"Robot is {'not' if has_drink else ''}carrying a drink. "
                        f"This is the expected state. Mass reading is {self.mass}"
                    )
                else:
                    rospy.loginfo(
                        f"Robot is {'not' if has_drink else ''}carrying a drink. "
                        f"This is NOT the expected state. Mass reading is {self.mass}"
                    )
            else:
                rospy.loginfo(f"Failed to get drink status")
            self.wait_for_drink_action_server.set_succeeded(result)
