import rospy
from typing import Optional
import py_trees
from trees.managers.drink_mission_manager import DrinkMissionManager


class WaitForMissionsBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, mission_manager: DrinkMissionManager, timeout: Optional[float] = None):
        self.mission_manager = mission_manager
        self.timeout = rospy.Duration(timeout)
        self.start_time = rospy.Time(0.0)
        super().__init__(f"Wait for missions")
    
    def initialise(self):
        self.start_time = rospy.Time.now()
        return super().initialise()
    
    def update(self):
        if self.timeout is not None and rospy.Time.now() - self.start_time > self.timeout:
            rospy.logwarn("Timed out while waiting for missions")
            return py_trees.Status.FAILURE
        if self.mission_manager.get_active() is not None:
            return py_trees.Status.SUCCESS
        else:
            return py_trees.Status.RUNNING
