import rospy
import py_trees

from geometry_msgs.msg import Twist


class RotateInPlaceBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, angular_velocity: float, timeout: rospy.Duration, topic="/bw/cmd_vel"):
        self.angular_velocity = angular_velocity
        self.timeout = timeout
        self.start_time = rospy.Time()
        self.cmd_vel_pub = rospy.Publisher(topic, Twist, queue_size=10)
        super().__init__("Rotate Indefinitely")
    
    def initialise(self):
        self.start_time = rospy.Time.now()
    
    def update(self):
        if rospy.Time.now() - self.start_time > self.timeout:
            return py_trees.Status.FAILURE
        twist = Twist()
        twist.angular.z = self.angular_velocity
        self.cmd_vel_pub(twist)
        return py_trees.Status.RUNNING
