import rospy

from bw_interfaces.srv import TeachWaypoint, TeachWaypointRequest, TeachWaypointResponse


class TeachWaypointManager:
    def __init__(self) -> None:
        self.teach_waypoint = rospy.ServiceProxy("/bw/teach_waypoint", TeachWaypoint)

    def teach(self, name: str) -> bool:
        result: TeachWaypointResponse = self.teach_waypoint(TeachWaypointRequest(name=name))
        return result.success
