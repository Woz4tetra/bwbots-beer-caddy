import rospy
import py_trees

from bw_interfaces.msg import ChargeState

class IsChargingBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, topic="charger", timeout=3.0):
        super().__init__("Is charging")
        self.is_charging_sub = rospy.Subscriber(topic, ChargeState, self.charger_callback, queue_size=10)
        self.charger_state = ChargeState()
        self.state_timestamp = rospy.Time(0)
        self.start_time = rospy.Time(0)
        self.timeout = rospy.Duration(timeout)

    def initialise(self):
        self.start_time = rospy.Time.now()

    def charger_callback(self, msg):
        self.state_timestamp = rospy.Time.now()
        self.charger_state = msg
    
    def update(self):
        if rospy.Time.now() - self.start_time > self.timeout:
            return py_trees.Status.FAILURE
        is_charging = self.charger_state.is_charging or self.charger_state.battery_voltage > 12.2
        if is_charging and rospy.Time.now() - self.state_timestamp < self.timeout:
            return py_trees.Status.SUCCESS
        return py_trees.Status.RUNNING
