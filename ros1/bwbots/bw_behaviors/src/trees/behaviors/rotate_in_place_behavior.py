import rospy
import py_trees

from geometry_msgs.msg import Twist


class RotateInPlaceBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, angular_velocity: float, timeout: float, topic="cmd_vel"):
        self.angular_velocity = angular_velocity
        self.timeout = rospy.Duration(timeout)
        self.start_time = rospy.Time()
        self.cmd_vel_pub = rospy.Publisher(topic, Twist, queue_size=10)
        super().__init__("Rotate in place")

    def initialise(self):
        self.start_time = rospy.Time.now()
        rospy.loginfo(f"Rotating for {self.timeout.to_sec()} seconds")
    
    def publish_rotation(self, angular_velocity: float):
        twist = Twist()
        twist.angular.z = angular_velocity
        self.cmd_vel_pub.publish(twist)
    
    def update(self):
        if rospy.Time.now() - self.start_time > self.timeout:
            rospy.loginfo(f"Finished rotating for {self.timeout.to_sec()} seconds")
            self.publish_rotation(0.0)
            return py_trees.Status.SUCCESS
        self.publish_rotation(self.angular_velocity)
        return py_trees.Status.RUNNING
