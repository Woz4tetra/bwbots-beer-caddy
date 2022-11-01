#!/usr/bin/env python3
import os
import yaml
import math
from collections import OrderedDict 

import rospy

import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseArray
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

from std_msgs.msg import ColorRGBA

from std_srvs.srv import Trigger, TriggerResponse

from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

from bw_interfaces.srv import GetAllWaypoints, GetAllWaypointsResponse
from bw_interfaces.srv import GetWaypoint, GetWaypointResponse
from bw_interfaces.srv import DeleteWaypoint, DeleteWaypointResponse
from bw_interfaces.srv import SavePose, SavePoseResponse
from bw_interfaces.srv import SaveRobotPose, SaveRobotPoseResponse
from bw_interfaces.srv import SaveTF, SaveTFResponse

from bw_interfaces.msg import Waypoint, WaypointArray

from bw_tools.waypoints import WaypointsManager


class BwWaypoints(WaypointsManager):
    def __init__(self):
        super().__init__()

        self.node_name = "bw_waypoints"
        rospy.init_node(
            self.node_name
            # disable_signals=True
            # log_level=rospy.DEBUG
        )
        waypoints_path_param = rospy.get_param("~waypoints_path", "~/.ros/waypoints")
        waypoints_path_param = os.path.expanduser(waypoints_path_param)
        waypoints_path_param += ".yaml"
        self.waypoints_path = waypoints_path_param

        self.map_frame = rospy.get_param("~map", "map")
        self.base_frame = rospy.get_param("~base_link", "base_link")
        self.marker_size = rospy.get_param("~marker_size", 0.25)
        self.marker_color = rospy.get_param("~marker_color", (0.0, 0.0, 1.0, 1.0))
        assert (type(self.marker_color) == tuple or type(self.marker_color) == list), "type(%s) != tuple or list" % type(self.marker_color)
        assert len(self.marker_color) == 4, "len(%s) != 4" % len(self.marker_color)

        self.load_from_path(self.waypoints_path)

        self.markers = MarkerArray()
        self.marker_poses = OrderedDict()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.marker_pub = rospy.Publisher("waypoint_markers", MarkerArray, queue_size=25)
        self.waypoints_pub = rospy.Publisher("waypoints", WaypointArray, queue_size=25)

        self.reload_waypoints_srv = self.create_service("reload_waypoints", Trigger, self.reload_waypoints_callback)
        self.get_all_waypoints_srv = self.create_service("get_all_waypoints", GetAllWaypoints, self.get_all_waypoints_callback)
        self.get_waypoint_srv = self.create_service("get_waypoint", GetWaypoint, self.get_waypoint_callback)
        self.delete_waypoint_srv = self.create_service("delete_waypoint", DeleteWaypoint, self.delete_waypoint_callback)
        self.save_pose_srv = self.create_service("save_pose", SavePose, self.save_pose_callback)
        self.save_tf_srv = self.create_service("save_tf", SaveTF, self.save_tf_callback)
        self.save_robot_pose_srv = self.create_service("save_robot_pose", SaveRobotPose, self.save_robot_pose_callback)

        rospy.loginfo("%s is ready" % self.node_name)

    # ---
    # Service callbacks
    # ---

    def get_all_waypoints_callback(self, req):
        return GetAllWaypointsResponse(self.get_waycoords_as_waypoints_array())

    def get_waypoint_callback(self, req):
        if not self.is_waypoint(req.name):
            return False

        return GetWaypointResponse(self.get_waycoord_as_waypoint(req.name))
    
    def delete_waypoint_callback(self, req):
        if not self.is_waypoint(req.name):
            return False

        success = self.pop_waycoord(req.name, self.waypoints_path)
        return DeleteWaypointResponse(success)

    def save_pose_callback(self, req):
        success = self.save_from_pose(req.name, req.waypoint)
        return SavePoseResponse(success)

    def save_robot_pose_callback(self, req):
        success = self.save_from_current(req.name)
        return SaveRobotPoseResponse(success)

    def save_tf_callback(self, req):
        success = self.save_from_tf(req.name, req.frame)
        return SaveTFResponse(success)

    def reload_waypoints_callback(self, req):
        if self.load_from_path(self.waypoints_path):
            return TriggerResponse(True, self.waypoints_path)
        else:
            return TriggerResponse(False, self.waypoints_path)

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
    
    def listen_for_service(self, name, srv_type):
        service_name = name + "_service_name"
        self.__dict__[service_name] = name
        rospy.loginfo("Waiting for service %s" % name)

        srv_obj = rospy.ServiceProxy(name, srv_type)
        rospy.loginfo("%s service is ready" % name)
        return srv_obj

    # ---
    # Node methods
    # ---

    def is_waypoint(self, name: str) -> bool:
        result = super().is_waypoint(name)
        if name not in self.marker_poses:
            rospy.logwarn("Waypoint name %s was added, but wasn't a registered marker! Adding." % name)
            pose = self.get_waypoint_pose(self.map_frame, self.get_waycoord_as_waypoint(name))
            self.add_marker(name, pose)
        return result
    
    def save_from_pose(self, name: str, pose: Pose) -> bool:
        result = super().save_from_pose(name, pose, self.waypoints_path)
        self.add_marker(name, pose)
        return result

    def save_from_current(self, name):
        # name: str, name of waypoint
        # returns: bool, whether the file was successfully written to and whether the tf lookup was successful
        return self.save_from_tf(name, self.base_frame)

    def save_from_tf(self, name, frame):
        # name: str, name of waypoint
        # returns: bool, whether the file was successfully written to and whether the tf lookup was successful
        try:
            current_tf = self.tf_buffer.lookup_transform(self.map_frame, frame, rospy.Time(0), rospy.Duration(1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("Failed to look up %s to %s. %s" % (self.map_frame, frame, e))
            return False
        
        pose = Pose()
        pose.position = current_tf.transform.translation
        pose.orientation = current_tf.transform.rotation
        
        return self.save_from_pose(name, pose)

    # ---
    # Waypoint visualization
    # ---

    def all_waypoints_to_markers(self):
        self.marker_poses = OrderedDict()
        for name, waypoint in self.waycoords.items():
            self.marker_poses[name] = self.waycoord_to_pose_stamped(waypoint, self.map_frame)
        self.update_markers()

    def add_marker(self, name, pose):
        self.marker_poses[name] = pose
        self.update_markers()
    
    def delete_marker(self, name):
        self.marker_poses.pop(name)
        self.update_markers()
    
    def update_markers(self):
        self.markers = MarkerArray()
        for name, pose in self.marker_poses.items():
            position_marker = self.make_marker(name, pose)
            text_marker = self.make_marker(name, pose)
            
            self.prep_position_marker(position_marker)
            
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.ns = "text" + text_marker.ns
            text_marker.text = name
            text_marker.scale.x = 0.0
            text_marker.scale.y = 0.0

            self.markers.markers.append(position_marker)
            self.markers.markers.append(text_marker)
    
    def prep_position_marker(self, position_marker):
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
    
    def make_marker(self, name, pose):
        # name: str, marker name
        # pose: PoseStamped
        marker = Marker()
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.header.frame_id = self.map_frame
        marker.lifetime = rospy.Duration(1.0)  # seconds
        marker.ns = name
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

    def publish_markers(self):
        if len(self.markers.markers) != 0:
            self.marker_pub.publish(self.markers)

    def publish_waypoints(self):
        self.waypoints_pub.publish(self.get_waycoords_as_waypoints_array())

    # ---
    # Run
    # ---

    def run(self):
        rate = rospy.Rate(3.0)
        while not rospy.is_shutdown():
            self.publish_markers()
            self.publish_waypoints()
            rate.sleep()


def main():
    node = BwWaypoints()
    try:
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.loginfo("Exiting %s node" % node.node_name)

if __name__ == "__main__":
    main()
