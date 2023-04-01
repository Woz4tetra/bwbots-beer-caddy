#!/usr/bin/env python3
import os
from typing import Optional
import yaml

import rospy
import tf2_ros

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Point

from std_msgs.msg import ColorRGBA

from bw_interfaces.srv import GetAllWaypoints, GetAllWaypointsResponse
from bw_interfaces.srv import GetWaypoint, GetWaypointResponse
from bw_interfaces.srv import GetWaypoints, GetWaypointsResponse
from bw_interfaces.srv import DeleteWaypoint, DeleteWaypointResponse
from bw_interfaces.srv import SaveWaypoint, SaveWaypointResponse
from bw_interfaces.srv import SaveTF, SaveTFResponse

from bw_interfaces.msg import Waypoint, WaypointArray

from bw_tools.waypoint import Waypoint2d, Waypoints2dArray

class BwWaypoints:
    def __init__(self):
        self.node_name = "bw_waypoints"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )

        self.waypoints_path = rospy.get_param("~waypoints_path", "~/.ros/waypoints")

        self.map_frame = rospy.get_param("~map", "map")
        self.base_frame = rospy.get_param("~base_link", "base_link")

        self.marker_size = rospy.get_param("~marker_size", 0.25)
        self.marker_color = rospy.get_param("~marker_color", (0.0, 0.0, 1.0, 1.0))

        self.waypoints = self.load_from_path(self.waypoints_path)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.marker_pub = rospy.Publisher("waypoint_markers", MarkerArray, queue_size=25)
        self.waypoints_pub = rospy.Publisher("waypoints", WaypointArray, queue_size=25)

        self.get_all_waypoints_srv = self.create_service("get_all_waypoints", GetAllWaypoints, self.get_all_waypoints_callback)
        self.get_waypoint_srv = self.create_service("get_waypoint", GetWaypoint, self.get_waypoint_callback)
        self.get_waypoints_srv = self.create_service("get_waypoints", GetWaypoints, self.get_waypoints_callback)
        self.delete_waypoint_srv = self.create_service("delete_waypoint", DeleteWaypoint, self.delete_waypoint_callback)
        self.save_pose_srv = self.create_service("save_waypoint", SaveWaypoint, self.save_waypoint_callback)
        self.save_tf_srv = self.create_service("save_tf", SaveTF, self.save_tf_callback)

        rospy.loginfo("%s is ready" % self.node_name)

    # ---
    # File manipulations
    # ---

    def load_from_path(self, waypoints_path) -> WaypointArray:
        waypoints_path = self.process_path(waypoints_path)
        if not os.path.isfile(waypoints_path):
            rospy.loginfo(f"Creating waypoints file: {waypoints_path}")
            self.save_to_path(waypoints_path, WaypointArray())
        with open(waypoints_path) as file:
            config = yaml.safe_load(file)
        waypoints2d_array = Waypoints2dArray()
        for data in config:
            waypoint2d = Waypoint2d(
                data["name"],
                data["parent_frame"],
                data["x"],
                data["y"],
                data["theta"],
            )
            waypoints2d_array.set(data["name"], waypoint2d)
        waypoints = waypoints2d_array.to_waypoints_array()
        rospy.loginfo(f"Loaded {len(waypoints.waypoints)} waypoints from {waypoints_path}")
        return waypoints

    def save_to_path(self, waypoints_path: str, waypoints: WaypointArray):
        waypoints_path = self.process_path(waypoints_path)
        waypoints_dir = os.path.dirname(waypoints_path)
        if not os.path.isdir(waypoints_dir):
            os.makedirs(waypoints_dir)
        with open(waypoints_path, 'w') as file:
            waypoints_list = []
            for waypoint in waypoints.waypoints:
                waypoint_dict = Waypoint2d.from_ros_waypoint(waypoint).to_dict()
                waypoints_list.append(waypoint_dict)
            yaml.safe_dump(waypoints_list, file)
            rospy.loginfo(f"Saving {len(waypoints.waypoints)} waypoints to {waypoints_path}")
    
    def process_path(self, waypoints_path):
        map_name = os.path.basename(waypoints_path)
        waypoints_dir = os.path.dirname(waypoints_path)
        if len(waypoints_dir) == 0:
            waypoints_dir = os.path.expanduser("~/.ros")
        waypoints_name = os.path.splitext(map_name)[0]
        waypoints_name += ".yaml"
        waypoints_path = os.path.join(waypoints_dir, waypoints_name)
        return waypoints_path

    # ---
    # Service creation macros
    # ---

    def create_service(self, name, srv_type, callback):
        name = self.node_name + "/" + name
        service_name = name + "_service_name"
        self.__dict__[service_name] = name
        rospy.loginfo("Setting up service %s" % name)

        srv_obj = rospy.Service(name, srv_type, callback)
        rospy.loginfo("%s service is ready" % name)
        return srv_obj

    # ---
    # Message manipulations
    # ---

    def get_waypoint(self, name: str) -> Optional[Waypoint]:
        for waypoint in self.waypoints.waypoints:
            if waypoint.name == name:
                return waypoint
        return None

    def delete_waypoint(self, name: str) -> bool:
        new_waypoints = list(filter(lambda w: w.name != name, self.waypoints.waypoints))
        success = len(new_waypoints) < len(self.waypoints.waypoints)
        self.waypoints.waypoints = new_waypoints
        return success
    
    def add_waypoint(self, waypoint: Waypoint) -> None:
        if self.get_waypoint(waypoint.name):
            self.delete_waypoint(waypoint.name)
        self.waypoints.waypoints.append(waypoint)

    # ---
    # Service callbacks
    # ---

    def get_all_waypoints_callback(self, req):
        self.waypoints = self.load_from_path(self.waypoints_path)
        return GetAllWaypointsResponse(self.waypoints)

    def get_waypoint_callback(self, req):
        self.waypoints = self.load_from_path(self.waypoints_path)
        waypoint = self.get_waypoint(req.name)
        if waypoint is None:
            return GetWaypointResponse(Waypoint(), False)
        else:
            return GetWaypointResponse(waypoint, True)
    
    def get_waypoints_callback(self, req):
        self.waypoints = self.load_from_path(self.waypoints_path)
        success = True
        array = WaypointArray()
        for name in req.names:
            waypoint = self.get_waypoint(name)
            if waypoint is None:
                success = False
            else:
                array.waypoints.append(waypoint)
        return GetWaypointsResponse(array, success)

    def delete_waypoint_callback(self, req):
        self.waypoints = self.load_from_path(self.waypoints_path)
        success = self.delete_waypoint(req.name)
        self.save_to_path(self.waypoints_path, self.waypoints)
        return DeleteWaypointResponse(success)

    def save_waypoint_callback(self, req):
        self.add_waypoint(req.waypoint)
        self.save_to_path(self.waypoints_path, self.waypoints)
        return SaveWaypointResponse(True)

    def save_tf_callback(self, req):
        try:
            current_tf = self.tf_buffer.lookup_transform(req.parent_frame, req.child_frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (req.parent_frame, req.child_frame, e))
            return SaveTFResponse(False)
        
        waypoint = Waypoint()
        waypoint.header.frame_id = req.parent_frame
        waypoint.name = req.name
        waypoint.pose.position = current_tf.transform.translation
        waypoint.pose.orientation = current_tf.transform.rotation
        self.add_waypoint(waypoint)
        self.save_to_path(self.waypoints_path, self.waypoints)
        return SaveTFResponse(True)

    # Waypoint markers

    def to_markers(self, waypoints: WaypointArray) -> MarkerArray:
        markers = MarkerArray()
        for waypoint in waypoints.waypoints:
            position_marker = self.make_marker(waypoint)
            text_marker = self.make_marker(waypoint)
            
            self.prep_position_marker(position_marker)
            
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.ns = "text" + text_marker.ns
            text_marker.text = waypoint.name
            text_marker.scale.x = 0.0
            text_marker.scale.y = 0.0

            markers.markers.append(position_marker)
            markers.markers.append(text_marker)
        return markers

    def prep_position_marker(self, position_marker: Marker):
        position_marker.type = Marker.ARROW
        position_marker.ns = "pos" + position_marker.ns
        position_marker.color.a = 0.75
        position_marker.scale.x = self.marker_size / 4.0
        position_marker.scale.y = self.marker_size / 2.5
        position_marker.scale.z = self.marker_size / 2.0
        
        p1 = Point()
        p2 = Point()
        
        p2.x = self.marker_size

        position_marker.points.append(p1)
        position_marker.points.append(p2)
    
    def make_marker(self, waypoint: Waypoint) -> Marker:
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = waypoint.pose
        marker.header.frame_id = waypoint.header.frame_id
        marker.lifetime = rospy.Duration(1.0)  # seconds
        marker.ns = waypoint.name
        marker.id = 0  # all waypoint names should be unique

        scale_vector = Vector3()
        scale_vector.x = self.marker_size
        scale_vector.y = self.marker_size
        scale_vector.z = self.marker_size
        marker.scale = scale_vector
        marker.color = ColorRGBA(
            r=self.marker_color[0],
            g=self.marker_color[1],
            b=self.marker_color[2],
            a=self.marker_color[3],
        )

        return marker

    def run(self):
        rate = rospy.Rate(3.0)
        while not rospy.is_shutdown():
            self.waypoints_pub.publish(self.waypoints)
            self.marker_pub.publish(self.to_markers(self.waypoints))
            rate.sleep()


def main():
    node = BwWaypoints()
    node.run()


if __name__ == "__main__":
    main()
