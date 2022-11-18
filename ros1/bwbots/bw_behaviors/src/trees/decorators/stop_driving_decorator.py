import rospy
import py_trees

from geometry_msgs.msg import Twist


class StopDrivingDecorator(py_trees.decorators.Decorator):
    def __init__(self, child, topic="/bw/cmd_vel"):
        self.start_time = rospy.Time()
        self.cmd_vel_pub = rospy.Publisher(topic, Twist, queue_size=10)
        super().__init__(child, py_trees.common.Name.AUTO_GENERATED)
    
    def update(self):
        result = super().update()
        if result == py_trees.Status.FAILURE or result == py_trees.Status.SUCCESS:
            self.cmd_vel_pub(Twist())
        return result
