from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container
from bw_interfaces.msg import GoToPoseGoal


class SimpleGoToPose(Behaviour):
    def __init__(self, container: Container, goal: GoToPoseGoal):
        super().__init__(self.__class__.__name__)
        self.goal = goal
        self.go_to_pose = container.simple_go_to_pose

    def initialise(self) -> None:
        self.go_to_pose.send_goal(self.goal)

    def update(self) -> Status:
        return self.go_to_pose.status
