import rospy
import py_trees


class PauseBeforeRunning(py_trees.decorators.Decorator):
    def __init__(self, child, delay_seconds: float):
        self.delay = rospy.Duration(delay_seconds)
        self.delay_start = rospy.Time(0)
        super().__init__(child, py_trees.common.Name.AUTO_GENERATED)
    
    def initialise(self):
        self.delay_start = rospy.Time.now()
        return super().initialise()
    
    def update(self):
        if rospy.Time.now() - self.delay_start < self.delay:
            return py_trees.Status.RUNNING
        else:
            return self.decorated.status
