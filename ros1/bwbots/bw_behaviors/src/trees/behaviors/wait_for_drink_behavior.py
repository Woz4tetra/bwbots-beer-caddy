import rospy
import py_trees
import py_trees_ros

from bw_interfaces.msg import WaitForDrinkAction, WaitForDrinkGoal, WaitForDrinkResult


class WaitForDrinkBehavior(py_trees_ros.actions.ActionClient):
    def __init__(
        self, timeout: float, mass_threshold: float, expected_state: bool = False
    ):
        goal = WaitForDrinkGoal()
        goal.timeout = rospy.Duration(timeout)  # type: ignore
        goal.mass_threshold = mass_threshold
        self.expected_state = expected_state

        super().__init__(
            "Wait for Drink",
            WaitForDrinkAction,
            goal,
            action_namespace="/bw/wait_for_drink",
        )

    def update(self):
        action_result = super().update()
        if action_result == py_trees.Status.SUCCESS:
            result: WaitForDrinkResult = self.action_client.get_result()  # type: ignore
            if result.success:
                status = result.state_matched
                if status == self.expected_state:
                    return py_trees.Status.SUCCESS
                else:
                    return py_trees.Status.FAILURE
            else:
                return py_trees.Status.FAILURE
        return action_result
