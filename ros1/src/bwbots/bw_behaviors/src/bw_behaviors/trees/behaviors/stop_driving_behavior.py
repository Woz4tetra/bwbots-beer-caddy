from typing import Optional
import rospy
import py_trees

from geometry_msgs.msg import Twist


class StopDrivingBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, topic="cmd_vel", pause: Optional[float] = None):
        self.topic = topic
        self.cmd_vel_pub = rospy.Publisher(self.topic, Twist, queue_size=10)
        self.start_time = rospy.Time()
        self.pause_duration = rospy.Duration(pause)  # type: ignore
        super().__init__("Stop Driving")

    def initialise(self):
        self.start_time = rospy.Time.now()

    def update(self):
        if self.cmd_vel_pub is None:
            raise RuntimeError("Failed to initialize velocity publisher")
        self.cmd_vel_pub.publish(Twist())
        if self.pause_duration is None:
            return py_trees.Status.SUCCESS
        else:
            if rospy.Time.now() - self.start_time > self.pause_duration:
                return py_trees.Status.SUCCESS
            else:
                return py_trees.Status.RUNNING
