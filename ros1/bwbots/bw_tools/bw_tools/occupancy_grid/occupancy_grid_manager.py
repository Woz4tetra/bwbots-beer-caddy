#!/usr/bin/env python
"""
Adapted from https://github.com/awesomebytes/occupancy_grid_python
Class to deal with OccupancyGrid in Python
as in local / global costmaps.

Author: Sammy Pfeiffer <Sammy.Pfeiffer at student.uts.edu.au>
        Ben Warwick
"""

import os
import cv2
import copy
import yaml
from nav_msgs.msg import OccupancyGrid
import numpy as np
from itertools import product

from bw_tools.robot_state.robot_state import State, Pose2d

class OccupancyGridManager:
    def __init__(self):
        # grid meta data
        self.map_config = {
            "resolution": 0.0,
            "width": 0,
            "height": 0,
            "origin": (0, 0),
        }

        # frame id of map
        self._reference_frame = ""

        # map data as array.
        # for maps, (defined by gmapping)
        #   0 = free
        #   100 = occupied
        #   -1 = unknown
        # for costmaps, (not defined by ROS. Convention defined here)
        #   0 = no cost
        #   100 = maximum cost
        #   -1 = unknown cost
        self.grid_data = np.array([], dtype=np.int8)

    @classmethod
    def from_ogm(cls, other: "OccupancyGridManager"):
        self = cls()
        self.map_config = copy.copy(other.map_config)
        self._reference_frame = other._reference_frame
        self.grid_data = np.copy(other.grid_data)
        return self

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
        return self

    @classmethod
    def from_map_file(cls, config_path, reference_frame="map", image_path=None):
        """
        This method creates an OccupancyGridManager using map file data.
        
        map files encode free, occupied, or unknown areas. When opening the image
        in a normal image viewer, the +Y axis is down and +X axis is to the right.
        To view map files, the image needs to be flipped along the X axis. This method
        does not flip the image.
        
        grid_data schema:
            free = 0
            occupied = 100
            unknown = -1
        map file schema (default gmapping output):
            free = 254 (occupied if negate is True)
            occupied = 0 (free if negate is True)
            unknown = 205
        map -> grid_data transform:
            convert BGR to gray
            bitwise not
            scale from 0..255 to 0..1
            label pixels < free_thresh as 0 (free)
            label pixels > occupied_thresh as 100 (occupied)
            label other pixels as -1 (unknown)
        """
        
        self = cls()
        with open(config_path) as file:
            map_file_config = yaml.safe_load(file)
        
        image = self._load_from_dict(map_file_config, reference_frame, config_path, image_path)

        occupied_thresh = map_file_config["occupied_thresh"]
        free_thresh = map_file_config["free_thresh"]
        negate = map_file_config["negate"] != 0
        if negate:
            swap = occupied_thresh
            occupied_thresh = free_thresh
            free_thresh = swap

        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = np.bitwise_not(image)
        image = image.astype(np.float32)
        max_value = np.iinfo(np.uint8).max
        image /= max_value
        self.grid_data = np.zeros(image.shape, dtype=np.int8)
        self.grid_data[np.where(image < free_thresh)] = 0
        self.grid_data[np.where(image > occupied_thresh)] = 100
        self.grid_data[np.where((image > free_thresh) & (image < occupied_thresh))] = -1

        return self
    
    @classmethod
    def from_costmap_file(cls, config_path, reference_frame="map", image_path=None):
        """
        This method creates an OccupancyGridManager using map file data interpreted as a cost map.
        
        cost map files encode cost from 0..100. When opening the image
        in a normal image viewer, the +Y axis is down and +X axis is to the right.
        To view map files, the image needs to be flipped along the X axis. This method
        does not flip the image.
        
        grid_data schema:
            no cost = 0
            maximum cost = 100
            unknown cost = -1
        map file schema (convension defined here):
            no cost = 0
            maximum cost = 100
            unknown = 101 (or higher)
        map -> grid_data transform:
            convert BGR to gray (uint8)
            label all pixels greater than 100 as -1 (unknown)
        """
        self = cls()
        with open(config_path) as file:
            map_file_config = yaml.safe_load(file)
        
        image = self._load_from_dict(map_file_config, reference_frame, config_path, image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image = np.bitwise_not(image)
        self.grid_data = image.astype(np.int8)
        self.grid_data[np.where(image > 100)] = -1
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
        msg.data = self.grid_data.astype(np.int8).flatten().tolist()
        return msg

    def _write_files(self, path, map_file_config, image):
        image_path = os.path.join(os.path.splitext(path)[0] + ".pgm")
        map_file_config["image"] = os.path.basename(image_path)

        with open(path, 'w') as file:
            yaml.dump(map_file_config, file)
        cv2.imwrite(image_path, image)

    def write_map(self, path, occupied_thresh=0.65, free_thresh=0.196, unknown=0.2, negate=False):
        """
        grid data -> map file friendly image
            scale from 0..100 to 0..255
            convert to uint8
            bitwise not
            any values in grid_data <= -1 set to 255 * (1 - unknown)
            any values in grid_data > 100 set to 255 * (1 - unknown)
        """
        
        map_file_config = {}
        map_file_config["resolution"] = self.resolution
        map_file_config["origin"] = self.origin.to_list()
        map_file_config["occupied_thresh"] = occupied_thresh
        map_file_config["free_thresh"] = free_thresh
        map_file_config["negate"] = int(negate)
        max_value = np.iinfo(np.uint8).max
        unknown_value = int(max_value * (1.0 - unknown))
        
        max_value = np.iinfo(np.uint8).max
        image = self.grid_data
        image = image.astype(np.float32)
        image = image * max_value / 100.0
        image = image.astype(np.uint8)
        image = np.bitwise_not(image)
        image[self.grid_data <= -1] = unknown_value
        image[self.grid_data > 100] = unknown_value

        self._write_files(path, map_file_config, image)

    def write_costmap(self, path):
        map_file_config = {}
        map_file_config["resolution"] = self.resolution
        map_file_config["origin"] = self.origin.to_list()
        map_file_config["occupied_thresh"] = 0.65  # unused by costmap
        map_file_config["free_thresh"] = 0.196  # unused by costmap
        map_file_config["negate"] = 0  # unused by costmap

        unknown_value = 0

        image = self.grid_data

        image = image.astype(np.uint8)
        image = np.bitwise_not(image)
        image[self.grid_data <= -1] = unknown_value
        image[self.grid_data > 100] = unknown_value

        self._write_files(path, map_file_config, image)


    @property
    def resolution(self):
        return self.map_config.get("resolution", 0.0)

    @property
    def width(self):
        return self.map_config.get("width", 0)

    @property
    def height(self):
        return self.map_config.get("height", 0)

    @property
    def origin(self):
        return self.map_config.get("origin", (0, 0))

    @property
    def reference_frame(self):
        return self._reference_frame
    
    def is_set(self):
        return len(self.map_config) != 0 and len(self._reference_frame) != 0 and len(self.grid_data) != 0

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
    
    def set_grid_data(self, grid_data):
        grid_data = grid_data.astype(np.int8)
        self.grid_data = grid_data

    def to_debug_image(self, unknown_color=(128, 128, 0)):
        max_value = np.iinfo(np.uint8).max
        image = self.grid_data.astype(np.float32)
        image = image / 100.0 * max_value
        image = image.astype(np.uint8)
        image = np.bitwise_not(image)
        if image is None:
            return np.array([], dtype=np.uint8)
        if image.size > 0:
            image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            image[np.where(self.grid_data < 0)] = np.array(unknown_color, dtype=np.uint8)
        else:
            return np.array([], dtype=np.uint8)
        image = np.flipud(image)
        image = np.ascontiguousarray(image, dtype=np.uint8)
        return image

    def get_world_x_y(self, costmap_x, costmap_y):
        world_x = costmap_x * self.resolution + self.origin.x
        world_y = (self.height - costmap_y) * self.resolution + self.origin.y
        return world_x, world_y

    def get_costmap_x_y(self, world_x, world_y):
        costmap_x = int(
            round((world_x - self.origin.x) / self.resolution))
        costmap_y = int(
            self.height - round((world_y - self.origin.y) / self.resolution))

        return costmap_x, costmap_y

    def get_cost_from_world_x_y(self, x, y):
        cx, cy = self.get_costmap_x_y(x, y)
        try:
            return self.get_cost_from_costmap_x_y(cx, cy)
        except IndexError as e:
            raise IndexError("Coordinates out of grid (in frame: {}) x: {}, y: {} must be in between: [{}, {}], [{}, {}]. Internal error: {}".format(
                self._reference_frame, x, y,
                self.origin.x,
                self.origin.x + self.width * self.resolution,
                self.origin.y,
                self.origin.y + self.height * self.resolution,
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

    def set_scale(self, scale):
        if scale == 1.0:
            return
        self.map_config["resolution"] /= scale
        self.map_config["width"] = int(self.map_config["width"] * scale)
        self.map_config["height"] = int(self.map_config["height"] * scale)
        self.grid_data = cv2.resize(self.grid_data, dsize=(self.map_config["width"], self.map_config["height"]), interpolation=cv2.INTER_NEAREST)

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