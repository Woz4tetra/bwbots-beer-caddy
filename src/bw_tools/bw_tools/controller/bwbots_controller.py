import math
import rospy
from typing import Tuple
from bw_tools.robot_state import Pose2d, Velocity

from bw_tools.controller.data import ControllerStateMachineConfig, ControllerState
from bw_tools.controller.controller import Controller
from bw_tools.controller.states import RotateToAngle, DriveToPose, ControllerBehavior


class BwbotsController(Controller):
    def __init__(self,
                config: ControllerStateMachineConfig) -> None:
        super().__init__(config)
        self.states = {
            ControllerState.ROTATE_IN_PLACE_START: RotateToAngle(
                config.settle_time,
                config.pose_tolerance.theta,
                config.rotate_trapezoid,
                self.compute_heading,
            ),
            ControllerState.DRIVE_TO_POSE: DriveToPose(
                config.rotate_while_driving,
                not config.rotate_in_place_end,
                config.settle_time,
                config.pose_tolerance,
                config.drive_to_pose_trapezoid,
                config.rotate_trapezoid,
                config.strafe_angle_threshold,
                self.compute_heading,
            ),
            ControllerState.ROTATE_IN_PLACE_END: RotateToAngle(
                config.settle_time,
                config.pose_tolerance.theta,
                config.rotate_trapezoid,
                self.compute_theta,
            ),
            ControllerState.IDLE: ControllerBehavior()
        }
        
        self.active_state = ControllerState.IDLE
        self.is_reversed = False

    def compute_heading(self, goal_pose: Pose2d, current_pose: Pose2d) -> float:
        heading = goal_pose.relative_to(current_pose).heading()
        if self.is_reversed:
            heading = Pose2d.normalize_theta(heading + math.pi)
        return heading

    def compute_theta(self, goal_pose: Pose2d, current_pose: Pose2d) -> float:
        return goal_pose.relative_to(current_pose).theta

    def get_behavior(self) -> ControllerBehavior:
        return self.states[self.active_state]
    
    def set_state(self, state: ControllerState, goal_pose: Pose2d, current_pose: Pose2d) -> None:
        self.get_behavior().deinitialize(goal_pose, current_pose)
        self.active_state = state
        self.get_behavior().initialize(goal_pose, current_pose)
        rospy.logdebug(f"Setting controller state to {state}. goal={goal_pose}, current={current_pose}")

    def compute(self, goal_pose: Pose2d, current_pose: Pose2d) -> Tuple[Velocity, bool]:
        behavior = self.get_behavior()
        velocity, is_done = behavior.compute(goal_pose, current_pose)
        is_state_machine_done = False
        rospy.logdebug(f"State: {self.active_state}. Velocity: {velocity}. Is done: {is_done}.")
        if is_done:
            if self.active_state == ControllerState.IDLE:
                self.is_reversed = (
                    self.config.allow_reverse and 
                    abs(self.compute_heading(goal_pose, current_pose)) > math.pi / 2.0
                )
                velocity = Velocity()
                if self.config.rotate_in_place_start:
                    rospy.logdebug("Starting controller by rotating in place.")
                    self.set_state(ControllerState.ROTATE_IN_PLACE_START, goal_pose, current_pose)
                else:
                    rospy.logdebug("Starting controller by driving to pose.")
                    self.set_state(ControllerState.DRIVE_TO_POSE, goal_pose, current_pose)
                rospy.logdebug(f"Controller is{'' if self.is_reversed else ' not'} reversed")
            elif self.active_state == ControllerState.ROTATE_IN_PLACE_START:
                rospy.logdebug("Rotate in place complete. Driving to pose.")
                self.set_state(ControllerState.DRIVE_TO_POSE, goal_pose, current_pose)
            elif self.active_state == ControllerState.DRIVE_TO_POSE:
                if self.config.rotate_in_place_end:
                    rospy.logdebug("Drive to pose complete. Rotating to final goal theta.")
                    self.set_state(ControllerState.ROTATE_IN_PLACE_END, goal_pose, current_pose)
                else:
                    rospy.logdebug("Drive to pose complete. Controller is finished.")
                    is_state_machine_done = True
            elif self.active_state == ControllerState.ROTATE_IN_PLACE_END:
                rospy.logdebug("Rotate in place complete. Controller is finished.")
                is_state_machine_done = True
                velocity = Velocity()
        # else:
        #     error = goal_pose.relative_to(current_pose)
        #     if (self.active_state == ControllerState.DRIVE_TO_POSE and
        #         (self.config.rotate_in_place_start or self.config.rotate_in_place_end) and
        #         abs(error.theta) > self.config.rotate_angle_threshold):
        #         rospy.logdebug("Rotated too far while driving to pose. Rotating in place.")
        #         self.set_state(ControllerState.ROTATE_IN_PLACE_START, goal_pose, current_pose)
        #     elif (self.active_state == ControllerState.ROTATE_IN_PLACE_END and 
        #         self.config.rotate_while_driving and
        #         error.magnitude() > self.config.pose_tolerance.magnitude()):
        #         rospy.logdebug("Drifted too far from the goal while rotating. Driving to pose.")
        #         self.set_state(ControllerState.DRIVE_TO_POSE, goal_pose, current_pose)

        return velocity, is_state_machine_done