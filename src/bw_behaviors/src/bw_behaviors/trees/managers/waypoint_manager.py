import rospy

from bw_interfaces.srv import GetWaypoints

class WaypointManager:
    def __init__(self, service_topic="/bw/bw_waypoints/get_waypoints") -> None:
        self.get_waypoints_srv = rospy.ServiceProxy(service_topic, GetWaypoints)
    
    def get_waypoint(self, name):
        rospy.loginfo(f"Requesting locations of waypoint: {name}")
        if self.get_waypoints_srv is None:
            rospy.logwarn("Waypoint service is not initialized!")
            return None
        result = self.get_waypoints_srv([name])
        if not result.success:
            rospy.logwarn("Failed to get waypoint locations")
            return None
        rospy.loginfo(f"Waypoint locations: {result}")
        waypoints_array = result.waypoints
        return waypoints_array
