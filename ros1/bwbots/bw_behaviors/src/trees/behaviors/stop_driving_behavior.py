from typing import Optional
import rospy
import py_trees

from geometry_msgs.msg import Twist


class StopDrivingBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, topic="/bw/cmd_vel", pause: Optional[rospy.Duration] = None):
        self.topic = topic
        self.cmd_vel_pub: Optional[Twist] = None
        self.start_time = rospy.Time()
        self.pause_duration = pause
        super().__init__("Stop Driving")
    
    def setup(self, timeout):
        self.cmd_vel_pub = rospy.Publisher(self.topic, Twist, queue_size=10)
        return super().setup(timeout)
    
    def initialise(self):
        self.start_time = rospy.Time.now()
    
    def update(self):
        if self.cmd_vel_pub is None:
            raise RuntimeError("Failed to initialize velocity publisher")
        self.cmd_vel_pub(Twist())
        if self.pause_duration is None:
            return py_trees.Status.SUCCESS        
        else:
            if rospy.Time.now() - self.start_time > self.pause_duration:
                return py_trees.Status.SUCCESS
            else:
                return py_trees.Status.RUNNING
