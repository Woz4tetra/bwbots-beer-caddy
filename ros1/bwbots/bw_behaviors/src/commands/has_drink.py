import rospy
import actionlib
from bw_interfaces.msg import LoadCell
from bw_interfaces.msg import HasDrinkAction, HasDrinkGoal, HasDrinkFeedback, HasDrinkResult


class HasDrinkCommand:
    def __init__(self) -> None:
        self.load_cell_sub = rospy.Subscriber("/bw/load_cell/mass", LoadCell, self.load_cell_callback, queue_size=10)
        self.prev_reading = rospy.Time(0)
        self.mass = 0.0

        self.action_server = actionlib.SimpleActionServer(
            "has_drink",
            HasDrinkAction,
            execute_cb=self.action_callback, 
            auto_start=False
        )
        self.action_server.start()
        rospy.loginfo("has_drink is ready")
    
    def load_cell_callback(self, msg):
        self.mass = msg.mass
        self.prev_reading = rospy.Time.now()

    def action_callback(self, goal: HasDrinkGoal):
        timeout = goal.timeout
        mass_threshold = goal.mass_threshold
        result = HasDrinkResult()
        result.success = False

        aborted = False
        start_time = rospy.Time.now()
        current_time = rospy.Time.now()
        while current_time - start_time < timeout:
            current_time = rospy.Time.now()
            if self.action_server.is_preempt_requested():
                aborted = True
                break
            if self.prev_reading > start_time:
                result.success = True
                result.has_drink = self.mass > mass_threshold
                break
            rospy.sleep(0.1)

        if aborted:
            self.action_server.set_aborted(result, "Interrupted while checking for a drink")
        else:
            if result.success:
                if result.has_drink:
                    rospy.loginfo(f"Robot is carrying a drink")
                else:
                    rospy.loginfo(f"Robot is not carrying a drink")
            else:
                rospy.loginfo(f"Failed to get drink status")
            self.action_server.set_succeeded(result)
