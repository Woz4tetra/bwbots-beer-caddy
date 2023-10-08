import rospy
from py_trees.behaviour import Behaviour
from py_trees.common import Status

from bw_behaviors.container import Container
from bw_tools.typing.basic import seconds_to_duration


class SetMotorsEnabled(Behaviour):
    def __init__(self, container: Container, motors_enabled: bool, timeout: float = 3.0):
        super().__init__(self.__class__.__name__)
        self.timeout = seconds_to_duration(timeout)
        self.motors_enabled_manager = container.motors_enabled_manager
        self.motors_enabled = motors_enabled
        self.timer = rospy.Time.now()

    def initialise(self) -> None:
        self.timer = rospy.Time.now()
        self.motors_enabled_manager.set_enable(self.motors_enabled)

    def update(self) -> Status:
        if rospy.Time.now() - self.timer > self.timeout:
            rospy.logwarn(f"Failed to set motor enable state to {self.motors_enabled}")
            return Status.FAILURE
        else:
            enabled_state = self.motors_enabled_manager.is_enabled()
            print("enabled_state: ", enabled_state)
            if enabled_state is None or enabled_state != self.motors_enabled:
                return Status.RUNNING
            else:
                return Status.SUCCESS
