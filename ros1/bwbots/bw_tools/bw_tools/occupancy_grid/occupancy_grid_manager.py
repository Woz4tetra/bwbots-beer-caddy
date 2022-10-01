#!/usr/bin/env python

import os
import cv2
import yaml
from nav_msgs.msg import OccupancyGrid
import numpy as np
from itertools import product
from geometry_msgs.msg import Pose

from bw_tools.robot_state import Pose2d, State

"""
Adapted from https://github.com/awesomebytes/occupancy_grid_python
Class to deal with OccupancyGrid in Python
as in local / global costmaps.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
"""


class OccupancyGridManager:
    def __init__(self):
        # OccupancyGrid starts on lower left corner
        self.map_config = {}
        self._reference_frame = ""
        self.grid_data = np.array([], dtype=np.int8)

    @classmethod
    def from_msg(cls, msg: OccupancyGrid):
        self = cls()
        # Contains resolution, width & height
        # np.set_printoptions(threshold=99999999999, linewidth=200)
        # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
        # first index is the row, second index the column
        self.grid_data = np.array(msg.data,
                                   dtype=np.int8).reshape(msg.info.height,
                                                          msg.info.width)
        self.map_config["resolution"] = msg.info.resolution
        self.map_config["width"] = msg.info.width
        self.map_config["height"] = msg.info.height
        self.map_config["origin"] = Pose2d.from_ros_pose(msg.info.origin)
        self._reference_frame = msg.header.frame_id

    @classmethod
    def from_map_file(cls, config_path, reference_frame="map", image_path=None):
        self = cls()
        with open(config_path) as file:
            map_file_config = yaml.safe_load(file)
        
        image = self._load_from_dict(map_file_config, reference_frame, config_path, image_path)

        max_value = np.iinfo(image.dtype).max
        occupied_thresh = int(map_file_config["occupied_thresh"] * max_value)
        free_thresh = int(map_file_config["free_thresh"] * max_value)
        negate = map_file_config["negate"] != 0
        if negate:
            image = cv2.bitwise_not(image)

        self.grid_data = np.zeros_like(image)
        self.grid_data = self.grid_data.astype(np.int8)
        self.grid_data[np.where(image < free_thresh)] = 100
        self.grid_data[np.where(image > occupied_thresh)] = 0
        self.grid_data[np.where((image > free_thresh) & (image < occupied_thresh))] = -1

        return self
    
    @classmethod
    def from_cost_file(cls, config_path, reference_frame="map", image_path=None):
        self = cls()
        with open(config_path) as file:
            map_file_config = yaml.safe_load(file)
        
        image = self._load_from_dict(map_file_config, reference_frame, config_path, image_path)

        max_value = np.iinfo(image.dtype).max

        data = np.bitwise_not(image)
        data = data.astype(np.float32)
        data = 100 * data / max_value
        self.grid_data = data.astype(np.int8)
        return self
    
    def _load_from_dict(self, map_file_config, reference_frame, config_path, image_path):
        self._reference_frame = reference_frame
        self.map_config["resolution"] = map_file_config["resolution"]
        self.map_config["origin"] = Pose2d(*map_file_config["origin"])

        if image_path is None:
            image_path = map_file_config["image"]
        if not image_path.startswith("/"):
            image_path = os.path.join(os.path.dirname(config_path), os.path.basename(image_path))
        if not os.path.isfile(image_path):
            raise FileNotFoundError("Map image file not found: %s" % str(image_path))
        image = cv2.imread(image_path)
        image = image.astype(np.uint8)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = np.flipud(image)
        
        self.map_config["width"] = image.shape[1]
        self.map_config["height"] = image.shape[0]
        return image

    def to_msg(self):
        msg = OccupancyGrid()
        msg.header.frame_id = self._reference_frame
        msg.info.resolution = self.resolution
        msg.info.width = self.width
        msg.info.height = self.height
        msg.info.origin = self.origin.to_ros_pose()
        msg.data = self.grid_data.tobytes()
        return msg
    
    def to_file(self, path, occupied_thresh=0.9, free_thresh=0.1, negate=False):
        map_file_config = {}
        map_file_config["resolution"] = self.resolution
        map_file_config["origin"] = self.origin.to_list()
        map_file_config["occupied_thresh"] = occupied_thresh
        map_file_config["free_thresh"] = free_thresh
        map_file_config["negate"] = int(negate)

        image_path = os.path.join(os.path.splitext(path)[0] + ".pgm")
        map_file_config["image"] = os.path.basename(image_path)

        with open(path, 'w') as file:
            yaml.dump(map_file_config, file)
        
        max_value = np.iinfo(np.uint8).max
        unknown_value = int(max_value * (occupied_thresh + free_thresh) / 2.0)
        image = self.grid_data.astype(np.float32)
        image = image / 100.0 * max_value
        image = image.astype(np.uint8)
        image = cv2.bitwise_not(image)
        image[np.where(self.grid_data < 0)] = unknown_value
        image = np.flipud(image)
        cv2.imwrite(image_path, image)

    @property
    def resolution(self):
        return self.map_config["resolution"]

    @property
    def width(self):
        return self.map_config["width"]

    @property
    def height(self):
        return self.map_config["height"]

    @property
    def origin(self):
        return self.map_config["origin"]

    @property
    def reference_frame(self):
        return self._reference_frame
    
    def set_resolution(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self.map_config["resolution"] = value

    def set_width(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self.map_config["width"] = value

    def set_height(self, value):
        assert isinstance(value, float) or isinstance(value, int)
        self.map_config["height"] = value

    def set_origin(self, value):
        if isinstance(value, list) or isinstance(value, tuple):
            assert len(value) == 2
            self.map_config["origin"] = Pose2d(value[0], value[1])
        elif isinstance(value, dict):
            assert "x" in value
            assert "y" in value
            self.map_config["origin"] = Pose2d(value["x"], value["y"])
        elif isinstance(value, State):
            self.map_config["origin"] = value
        else:
            raise ValueError("Invalid type for origin: %s" % repr(value))

    def set_reference_frame(self, value):
        self._reference_frame = value
    
    def set_image(self, grid_data):
        self.grid_data = grid_data
    
    def get_image(self):
        max_value = np.iinfo(np.uint8).max
        image = self.grid_data.astype(np.float32)
        image = image / 100.0 * max_value
        image = image.astype(np.uint8)
        image = cv2.bitwise_not(image)
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        image[np.where(self.grid_data < 0)] = np.array([128, 128, 0])
        image = image.astype(np.uint8)
        return image

    def get_world_x_y(self, costmap_x, costmap_y):
        world_x = costmap_x * self.resolution + self.origin.x
        world_y = costmap_y * self.resolution + self.origin.y
        return world_x, world_y

    def get_costmap_x_y(self, world_x, world_y):
        costmap_x = int(
            round((world_x - self.origin.x) / self.resolution))
        costmap_y = int(
            round((world_y - self.origin.y) / self.resolution))

        return costmap_x, costmap_y

    def get_cost_from_world_x_y(self, x, y):
        cx, cy = self.get_costmap_x_y(x, y)
        try:
            return self.get_cost_from_costmap_x_y(cx, cy)
        except IndexError as e:
            raise IndexError("Coordinates out of grid (in frame: {}) x: {}, y: {} must be in between: [{}, {}], [{}, {}]. Internal error: {}".format(
                self._reference_frame, x, y,
                self.origin.x,
                self.origin.x + self.height * self.resolution,
                self.origin.y,
                self.origin.y + self.width * self.resolution,
                e))

    def get_cost_from_costmap_x_y(self, x, y):
        if self.is_in_gridmap(x, y):
            # data comes in row-major order http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
            # first index is the row, second index the column
            return self.grid_data[y][x]
        else:
            raise IndexError(
                "Coordinates out of gridmap, x: {}, y: {} must be in between: [0, {}], [0, {}]".format(
                    x, y, self.height, self.width))

    def is_in_gridmap(self, x, y):
        if -1 < x < self.width and -1 < y < self.height:
            return True
        else:
            return False

    def get_closest_cell_under_cost(self, x, y, cost_threshold, max_radius):
        """
        Looks from closest to furthest in a circular way for the first cell
        with a cost under cost_threshold up until a distance of max_radius,
        useful to find closest free cell.
        returns -1, -1 , -1 if it was not found.

        :param x int: x coordinate to look from
        :param y int: y coordinate to look from
        :param cost_threshold int: maximum threshold to look for
        :param max_radius int: maximum number of cells around to check
        """
        return self._get_closest_cell_arbitrary_cost(
            x, y, cost_threshold, max_radius, bigger_than=False)

    def get_closest_cell_over_cost(self, x, y, cost_threshold, max_radius):
        """
        Looks from closest to furthest in a circular way for the first cell
        with a cost over cost_threshold up until a distance of max_radius,
        useful to find closest obstacle.
        returns -1, -1, -1 if it was not found.

        :param x int: x coordinate to look from
        :param y int: y coordinate to look from
        :param cost_threshold int: minimum threshold to look for
        :param max_radius int: maximum number of cells around to check
        """
        return self._get_closest_cell_arbitrary_cost(
            x, y, cost_threshold, max_radius, bigger_than=True)

    def _get_closest_cell_arbitrary_cost(self, x, y,
                                         cost_threshold, max_radius,
                                         bigger_than=False):

        # Check the actual goal cell
        try:
            cost = self.get_cost_from_costmap_x_y(x, y)
        except IndexError:
            return None

        if bigger_than:
            if cost > cost_threshold:
                return x, y, cost
        else:
            if cost < cost_threshold:
                return x, y, cost

        def create_radial_offsets_coords(radius):
            """
            Creates an ordered by radius (without repetition)
            generator of coordinates to explore around an initial point 0, 0

            For example, radius 2 looks like:
            [(-1, -1), (-1, 0), (-1, 1), (0, -1),  # from radius 1
            (0, 1), (1, -1), (1, 0), (1, 1),  # from radius 1
            (-2, -2), (-2, -1), (-2, 0), (-2, 1),
            (-2, 2), (-1, -2), (-1, 2), (0, -2),
            (0, 2), (1, -2), (1, 2), (2, -2),
            (2, -1), (2, 0), (2, 1), (2, 2)]
            """
            # We store the previously given coordinates to not repeat them
            # we use a Dict as to take advantage of its hash table to make it more efficient
            coords = {}
            # iterate increasing over every radius value...
            for r in range(1, radius + 1):
                # for this radius value... (both product and range are generators too)
                tmp_coords = product(range(-r, r + 1), repeat=2)
                # only yield new coordinates
                for i, j in tmp_coords:
                    if (i, j) != (0, 0) and not coords.get((i, j), False):
                        coords[(i, j)] = True
                        yield (i, j)

        coords_to_explore = create_radial_offsets_coords(max_radius)

        for idx, radius_coords in enumerate(coords_to_explore):
            # for coords in radius_coords:
            tmp_x, tmp_y = radius_coords
            # rospy.logdebug("Checking coords: " +
            #       str((x + tmp_x, y + tmp_y)) +
            #       " (" + str(idx) + " / " + str(len(coords_to_explore)) + ")")
            try:
                cost = self.get_cost_from_costmap_x_y(x + tmp_x, y + tmp_y)
            # If accessing out of grid, just ignore
            except IndexError:
                pass
            if bigger_than:
                if cost > cost_threshold:
                    return x + tmp_x, y + tmp_y, cost

            else:
                if cost < cost_threshold:
                    return x + tmp_x, y + tmp_y, cost

        return -1, -1, -1
