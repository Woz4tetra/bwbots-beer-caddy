import rospy
import py_trees
import py_trees_ros

from bw_interfaces.msg import HasDrinkAction, HasDrinkGoal, HasDrinkResult


class HasDrinkBehavior(py_trees_ros.actions.ActionClient):
    def __init__(self, timeout: float, mass_threshold: float, invert: bool = False):
        goal = HasDrinkGoal()
        goal.timeout = rospy.Duration(timeout)
        goal.mass_threshold = mass_threshold
        self.invert = invert

        super().__init__("Has Drink",
            HasDrinkAction,
            goal,
            action_namespace="/bw/has_drink")

    def update(self):
        action_result = super().update()
        if action_result == py_trees.Status.SUCCESS:
            result: HasDrinkResult = self.action_client.get_result()
            if result.success:
                status = result.has_drink
                if self.invert:
                    status = not status
                return py_trees.Status.SUCCESS if status else py_trees.Status.FAILURE
            else:
                return py_trees.Status.FAILURE
        return action_result
