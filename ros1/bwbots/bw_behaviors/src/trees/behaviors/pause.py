import rospy
import py_trees

class PauseBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, pause_seconds: float):
        self.pause_duration = rospy.Duration(pause_seconds)
        self.pause_start = rospy.Time(0)
        super().__init__(f"Pause for {pause_seconds} seconds")
     
    def initialise(self):
        self.pause_start = rospy.Time.now()
        rospy.loginfo(f"Pausing for {self.pause_duration.to_sec()} seconds")
        return super().initialise()
       
    def update(self):
        if rospy.Time.now() - self.pause_start < self.pause_duration:
            return py_trees.Status.RUNNING
        else:
            return py_trees.Status.SUCCESS
