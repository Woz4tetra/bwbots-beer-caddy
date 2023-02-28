import rospy
import py_trees

from bw_interfaces.msg import ChargeState

class IsChargingBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, topic="charger", sample_duration=1.0, current_threshold=0.03, invert=False):
        super().__init__(f"Is {'not ' if invert else ''}charging")
        self.is_charging_topic = topic
        self.start_time = rospy.Time(0)
        self.sample_duration = rospy.Duration(sample_duration)
        self.samples = []
        self.is_charging_sub = None
        self.invert = invert
        self.current_threshold = current_threshold

    def initialise(self):
        self.start_time = rospy.Time.now()
        self.samples = []
        self.is_charging_sub = rospy.Subscriber(self.is_charging_topic, ChargeState, self.charger_callback, queue_size=10)

    def charger_callback(self, msg):
        self.samples.append(msg.charge_current)
        rospy.logdebug(f"Current measurement: {msg.charge_current} A")
    
    def update(self):
        if rospy.Time.now() - self.start_time > self.sample_duration:
            if len(self.samples) == 0:
                return py_trees.Status.FAILURE
            else:
                current_average = sum(self.samples) / len(self.samples)
                state = current_average > self.current_threshold
                success = not state if self.invert else state
                rospy.logdebug(f"Current measurement average: {current_average} A. Is charging: {state}. Success: {success}")
                return py_trees.Status.SUCCESS if success else py_trees.Status.FAILURE
        return py_trees.Status.RUNNING

    def terminate(self, new_status):
        if self.is_charging_sub:
            self.is_charging_sub.unregister()
        return super().terminate(new_status)
