from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container


class TeachWaypoint(Behaviour):
    def __init__(self, container: Container, waypoint_name: str):
        super().__init__(self.__class__.__name__)
        self.waypoint_name = waypoint_name
        self.teach_waypoint_manager = container.teach_waypoint_manager

    def initialise(self) -> None:
        pass

    def update(self) -> Status:
        success = self.teach_waypoint_manager.teach(self.waypoint_name)
        return Status.SUCCESS if success else Status.FAILURE
