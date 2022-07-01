#!/usr/bin/env python3

import rospy
import time
from nav_msgs.msg import OccupancyGrid
from nav_map.msg import BoundingBox3DArray



class MapGenerator(object):
    def __init__(self):
        self.map_pub = 
        self.obs_sub = rospy.Subscriber()
        self.path_sub = rospy.Subscriber()
        self.map_file_path = rospy.get_param('map/file_path', "/home/administrator/catkin_ws/src/nav_map/excel_map/B5_envMap.csv")
        self.reference_frame = rospy.get_param('map/frame_id', "map")
        self.map = OccupancyGrid()
        self.width = rospy.get_param('map/width', 200)
        self.height = rospy.get_param('map/height', 200)
        self.resolution = rospy.get_param('map/resolution', 0.1)
        self.init_map
        self.obs_grid
        self.safety = rospy.get_param('map/safety_cells', 2)
        self.count_id = 0

    def obs_callback():
        pass

    def get_map():
        pass

    def get_grids():
        pass

    def restore_mao():
        pass

    def update_map():
        pass


