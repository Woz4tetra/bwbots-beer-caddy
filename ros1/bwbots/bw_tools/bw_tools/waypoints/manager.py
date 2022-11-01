import os
from typing import Optional
import yaml
import rospy
from collections import OrderedDict
from tf.transformations import euler_from_quaternion, quaternion_from_euler

from bw_interfaces.msg import Waypoint, WaypointArray
from geometry_msgs.msg import Pose, PoseArray, PoseStamped


class WaypointsManager:
    def __init__(self, waypoint_config: Optional[dict] = None) -> None:
        self.waycoords = OrderedDict() if waypoint_config is None else OrderedDict(waypoint_config)

    @classmethod
    def from_waypoint_array(cls, waypoint_array: WaypointArray) -> "WaypointsManager":
        self = cls()
        self.set_from_waypoint_array_msg(waypoint_array)
        return self

    def set_from_waypoint_array_msg(self, waypoint_array: WaypointArray) -> None:
        for waypoint in waypoint_array.waypoints:
            self.set_from_waypoint_msg(waypoint)

    def set_from_waypoint_msg(self, waypoint: Waypoint):
        self.waycoords[waypoint.name] = self.pose_to_waycoord(waypoint.pose)

    def get_waypoint_pose(self, waypoint_frame: str, waypoint: Waypoint) -> PoseStamped:
        name = waypoint.name
        if len(name) == 0:
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = waypoint_frame
            pose_stamped.pose = waypoint.pose
            return pose_stamped
        elif self.is_waypoint(name):
            pose_2d = self.get_waycoord(name)
            pose = self.waycoord_to_pose_stamped(pose_2d, waypoint_frame)
            return pose
        else:
            raise ValueError("Waypoint name '%s' is not registered." % name)

    def get_waypoints_array_as_pose_array(self, waypoint_frame: str, waypoints: WaypointArray) -> PoseArray:
        return self.get_waypoints_as_pose_array(waypoint_frame, waypoints.waypoints)

    def get_waypoints_as_pose_array(self, waypoint_frame: str, waypoints: list[Waypoint]) -> PoseArray:
        pose_array = PoseArray()
        for waypoint in waypoints:
            pose_stamped = self.get_waypoint_pose(waypoint_frame, waypoint)
            if pose_stamped is None:
                continue
            pose_array.header = pose_stamped.header
            pose_array.poses.append(pose_stamped.pose)
        return pose_array

    def get_waycoord_as_waypoint(self, name: str) -> Waypoint:
        waycoord = self.waycoords[name]
        pose = self.waycoord_to_pose(waycoord)
        waypoint_msg = Waypoint()
        waypoint_msg.pose = pose
        waypoint_msg.name = name
        return waypoint_msg

    def get_waycoords_as_waypoints_array(self) -> WaypointArray:
        waypoint_array = WaypointArray()
        for name in self.get_all_names():
            waypoint_msg = self.get_waycoord_as_waypoint(name)
            waypoint_array.waypoints.append(waypoint_msg)
        return waypoint_array

    # ---
    # File manipulations
    # ---

    def load_from_path(self, waypoints_path: str) -> bool:
        waypoints_path = self.process_path(waypoints_path)
        if not self.initialize_file(waypoints_path):
            return False
        try:
            with open(waypoints_path) as file:
                config = yaml.safe_load(file)
            if config is None:
                self.waycoords = OrderedDict()
            else:
                self.waycoords = OrderedDict(config)
            return True
        except BaseException as e:
            rospy.logwarn("Failed to load waypoints file '%s'. %s" % (waypoints_path, e))
            return False
    
    def initialize_file(self, waypoints_path: str) -> bool:
        # If file doesn't exist, create directories and empty file
        if os.path.isfile(waypoints_path):
            return True
        waypoints_dir = os.path.dirname(waypoints_path)
        if not os.path.isdir(waypoints_dir):
            os.makedirs(waypoints_dir)
        with open(waypoints_path, 'w') as file:
            file.write("")
        rospy.logwarn("Waypoints file '%s' doesn't exist. Creating file." % waypoints_path)
        return False

    def process_path(self, waypoints_path: str) -> str:
        map_name = os.path.basename(waypoints_path)
        waypoints_dir = os.path.dirname(waypoints_path)
        if len(waypoints_dir) == 0:
            waypoints_dir = os.path.expanduser("~/.ros")
        waypoints_name = os.path.splitext(map_name)[0]
        waypoints_name += ".yaml"
        waypoints_path = os.path.join(waypoints_dir, waypoints_name)
        return waypoints_path

    def save_to_path(self, waypoints_path: str) -> bool:
        try:
            with open(waypoints_path, 'w') as file:
                for name, waypoint in self.waycoords.items():
                    yaml.safe_dump({name: waypoint}, file)
            return True
        except BaseException as e:
            rospy.logwarn("Failed to save waypoints file '%s'. %s" % (waypoints_path, e))
            return False
    
    # ---
    # Manager methods
    # ---

    def is_waypoint(self, name: str) -> bool:
        if name not in self.waycoords:
            return False
        return True

    def save_from_pose(self, name: str, pose: Pose, waypoints_path: str) -> bool:
        # name: str, name of waypoint
        # pose: Pose. Assumed to be in the same frame of the other waypoints
        # returns: bool, whether the file was successfully written to
        self.waycoords[name] = self.pose_to_waycoord(pose)
        return self.save_to_path(waypoints_path)

    def get_waycoord(self, name: str) -> list[float]:
        # name: str, name of waypoint
        # returns: list, [x, y, theta]
        return self.waycoords[name]
    
    def pop_waycoord(self, name: str, waypoints_path: str) -> bool:
        # name: str, name of waypoint
        # returns: list, [x, y, theta]
        self.waycoords.pop(name)
        return self.save_to_path(waypoints_path)

    def get_all_waycoords(self) -> list[list[float]]:
        # returns: list, [[x, y, theta], ...]
        return [waypoint for waypoint in self.waycoords.values()]
    
    def get_all_names(self) -> list[str]:
        # returns: list, [str, ...] waypoint names
        return [name for name in self.waycoords.keys()]

    # ---
    # Conversion methods
    # ---

    def pose_to_waycoord(self, pose: Pose) -> list[float]:
        # pose: Pose
        # returns: list, [x, y, theta]
        yaw = euler_from_quaternion([
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ])[2]
        return [pose.position.x, pose.position.y, yaw]

    def waycoord_to_pose(self, waycoord: list[float]) -> Pose:
        # waypoint: list, [x, y, theta]
        # returns: Pose
        quat = quaternion_from_euler(0.0, 0.0, waycoord[2])
        pose = Pose()
        pose.position.x = waycoord[0]
        pose.position.y = waycoord[1]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose

    def waycoord_to_pose_stamped(self, waycoord: list[float], waypoint_frame) -> PoseStamped:
        # waypoint: list, [x, y, theta]
        # returns: PoseStamped
        pose = PoseStamped()
        pose.header.frame_id = waypoint_frame
        pose.pose = self.waycoord_to_pose(waycoord)
        return pose

    def waycoords_to_pose_array(self, waypoints: list[list[float]], waypoint_frame) -> PoseArray:
        # waypoint: list, [[x, y, theta], ...]
        # returns: PoseArray
        pose_array = PoseArray()
        pose_array.header.frame_id = waypoint_frame
        for waypoint in waypoints:
            pose = self.waycoord_to_pose_stamped(waypoint, waypoint_frame)
            pose_array.poses.append(pose.pose)
        return pose_array
