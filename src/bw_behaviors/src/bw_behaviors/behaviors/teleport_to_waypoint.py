import rospy
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class TeleportToWaypoint(Behaviour):
    def __init__(self, container: Container, waypoint_name: str):
        super().__init__(self.__class__.__name__)
        self.waypoint_name = waypoint_name
        self.reset_localization_manager = container.reset_localization_manager
        self.waypoint_manager = container.waypoint_manager

    def initialise(self) -> None:
        pass

    def update(self) -> Status:
        waypoint = self.waypoint_manager.waypoints.get(self.waypoint_name)
        if waypoint is None:
            rospy.logwarn("Waypoint %s not registered", self.waypoint_name)
            return Status.FAILURE
        else:
            self.reset_localization_manager.reset_to_waypoint(waypoint.to_ros_waypoint())
            return Status.SUCCESS
