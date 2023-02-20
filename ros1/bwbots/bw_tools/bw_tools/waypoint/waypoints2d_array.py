from typing import Dict, Optional, Tuple

from .waypoint2d import Waypoint2d
from bw_interfaces.msg import Waypoint, WaypointArray


class Waypoints2dArray:
    def __init__(self, waypoints2d: Optional[Dict[str, Waypoint2d]] = None) -> None:
        self.waypoints2d = {} if waypoints2d is None else waypoints2d

    def __getitem__(self, key: str) -> Waypoint2d:
        return self.waypoints2d[key]
    
    def __setitem__(self, key: str, value: Waypoint2d):
        self.waypoints2d[key] = value

    def get_names(self) -> Tuple[str]:
        return tuple(self.waypoints2d.keys())

    def get(self, key: str, default=None):
        return self.waypoints2d.get(key, default)

    def set(self, key: str, value: Waypoint2d):
        self.waypoints2d[key] = value

    def to_waypoints_array(self) -> WaypointArray:
        array = WaypointArray()
        for waypoint2d in self.waypoints2d.values():
            waypoint: Waypoint = waypoint2d.to_ros_waypoint()
            array.waypoints.append(waypoint)
        return array

    @classmethod
    def from_waypoints_array(cls, waypoints: WaypointArray) -> "Waypoints2dArray":
        self = cls()
        for waypoint in waypoints.waypoints:
            waypoint2d: Waypoint2d = Waypoint2d.from_ros_waypoint(waypoint)
            self.waypoints2d[waypoint2d.name] = waypoint2d
        return self
