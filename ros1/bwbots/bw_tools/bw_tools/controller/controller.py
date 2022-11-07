from ..robot_state import Pose2d, Velocity


class Controller:
    def __init__(self) -> None:
        self.pose_error = Pose2d()
        self.pose_tolerance = Pose2d()
        self.enabled = True
    
    def at_reference(self) -> bool:
        return abs(self.pose_error.x) < self.pose_tolerance.x and \
            abs(self.pose_error.y) < self.pose_tolerance.y and \
            abs(self.pose_error.theta) < self.pose_tolerance.theta

    def reset(self):
        pass

    def set_tolerance(self, tolerance: Pose2d):
        """
        * Sets the pose error which is considered tolerance for use with atReference().
        *
        * @param tolerance The pose error which is tolerable.
        """
        self.pose_tolerance = tolerance

    def calculate(self, **kwargs) -> Velocity:
        pass

    def set_enabled(self, enabled: bool) -> None:
        """
        * Enables and disables the controller for troubleshooting problems. When calculate() is called on
        * a disabled controller, only feedforward values are returned.
        *
        * @param enabled If the controller is enabled or not.
        """
        self.enabled = enabled
