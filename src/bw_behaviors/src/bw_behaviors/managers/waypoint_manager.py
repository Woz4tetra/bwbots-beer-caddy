from threading import Lock

import rospy

from bw_interfaces.msg import WaypointArray
from bw_tools.waypoint import Waypoints2dArray


class WaypointManager:
    def __init__(self) -> None:
        self.waypoints_sub = rospy.Subscriber("waypoints", WaypointArray, self.waypoints_callback, queue_size=1)
        self._waypoints = Waypoints2dArray()
        self.lock = Lock()

    @property
    def waypoints(self) -> Waypoints2dArray:
        with self.lock:
            return self._waypoints

    def waypoints_callback(self, msg: WaypointArray) -> None:
        with self.lock:
            self._waypoints = Waypoints2dArray.from_waypoints_array(msg)
