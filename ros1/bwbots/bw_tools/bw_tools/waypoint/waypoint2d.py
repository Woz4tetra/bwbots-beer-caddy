from typing import Dict
import rospy

from ..robot_state import Pose2d

from bw_interfaces.msg import Waypoint


class Waypoint2d(Pose2d):
    def __init__(self, name: str, parent_frame: str, x=0.0, y=0.0, theta=0.0):
        self.name = name
        self.parent_frame = parent_frame
        super().__init__(x, y, theta)

    def to_ros_waypoint(self) -> Waypoint:
        waypoint = Waypoint()
        waypoint.pose.position.x = self.x
        waypoint.pose.position.y = self.y
        waypoint.pose.orientation = self.get_theta_as_quat()
        waypoint.header.frame_id = self.parent_frame
        waypoint.header.stamp = rospy.Time.now()
        waypoint.name = self.name
        return waypoint
    
    def to_dict(self) -> Dict:
        return {
            "name": self.name,
            "parent_frame": self.parent_frame,
            "x": self.x,
            "y": self.y,
            "theta": self.theta
        }
    
    @classmethod
    def from_ros_waypoint(cls, waypoint: Waypoint) -> "Waypoint2d":
        return cls(
            waypoint.name, 
            waypoint.header.frame_id,
            waypoint.pose.position.x,
            waypoint.pose.position.y,
            cls.theta_from_quat(waypoint.pose.orientation)
        )

    def __str__(self):
        return f"{self.__class__.__name__}("\
                f"{self.name}, "\
                f"{self.parent_frame}, "\
                f"x={self.x:0.4f}, "\
                f"y={self.y:0.4f}, "\
                f"theta={self.theta:0.4f})"

