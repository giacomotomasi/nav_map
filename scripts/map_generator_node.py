#!/usr/bin/env python3

import csv
import time
import csv
import rospy
from nav_msgs.msg import OccupancyGrid
import tf
from nav_map.msg import BoundingBox3DArray

class MapGenerator(object):
    def __init__(self):
        # declare pubs and subs
        self.map_pub = rospy.Publisher('updated_map', OccupancyGrid, queue_size=1)
        self.obs_sub = rospy.Subscriber('/boundingBoxArray', BoundingBox3DArray, self.obs_callback)
        #self.path_sub = rospy.Subscriber()
        # get parameters
        self.map_file_path = rospy.get_param('map/file_path', "/home/administrator/catkin_ws/src/nav_map/excel_map/B5_envMap.csv")
        self.reference_frame = rospy.get_param('map/frame_id', "map")
        self.width = rospy.get_param('map/width', 200)
        self.height = rospy.get_param('map/height', 200)
        self.resolution = rospy.get_param('map/resolution', 0.1)
        self.safety = rospy.get_param('map/safety_cells', 2)
        self.map = OccupancyGrid()
        self.init_map = []
        self.obs_grid = []
        self.count_id = 0
        # initialize and publish map
        self.get_map()
        self.map_pub.publish(self.map)

    def obs_callback(self, obs_msg):
        self.obs_grid = []
        n = len(obs_msg.bboxes)
        if (n>0):
            for i in range(n):
                ax = obs_msg.bboxes[i].center.position.x - (obs_msg.bboxes[i].size.x/2) # resolution is subtracked to compensate "bad" approximations
                ay = obs_msg.bboxes[i].center.position.y + (obs_msg.bboxes[i].size.y/2)
                bx = obs_msg.bboxes[i].center.position.x + (obs_msg.bboxes[i].size.x/2)
                by = ay
                cx = ax
                cy = obs_msg.bboxes[i].center.position.y - (obs_msg.bboxes[i].size.y/2) - self.resolution

                x = [ax,bx,cx]
                y = [ay,by,cy]

                self.get_grids(x,y)
                self.update_map()
            else:
                self.update_map()

    def get_map(self):
        # open the file
        mapp = open(self.map_file_path)
        map_reader = csv.reader(mapp)
        # heck if the file is open
        if not mapp.closed:
            for line in map_reader:
                for elem in line:
                    if (elem != ','):
                        self.map.data.append(int(elem))
                        self.init_map.append(int(elem))
            mapp.close()
        else:
            print("Failed To Open File")

    def get_grids(self, x, y):
        n = len(x) # same as len(y)
        obstacle = []
        for idx in range(n):
            if (x[idx] < 0):
                # CHECK TRUNCATION IN PYTHON
                cell_x = int((x[idx]-0.05)/self.resolution)+100 
            else:
                cell_x = int(x[idx]/self.resolution)+100
            
            if (y[idx] < 0):
                cell_y = int((y[idx]-0.05)/self.resolution)+100 
            else:
                cell_y = int(y[idx]/self.resolution)+100

            obstacle.append([cell_x, cell_y])

        for row in range(obstacle[2][1]-self.safety, obstacle[1][1]+self.safety+1):
            for col in range(obstacle[0][0]-self.safety, obstacle[1][0]+self.safety+1):
                idxx = col + row*self.width
                self.obs_grid.append(idxx)

    def restore_map(self):
        print("a")
        # restore map to initial map
        self.map.data = self.init_map

    def update_map(self):
        n = len(self.obs_grid)

        self.map.header.seq = self.count_id
        self.map.header.stamp = rospy.Time.now()
        self.map.header.frame_id = self.reference_frame
        self.map.info.resolution = self.resolution
        self.map.info.width = self.width
        self.map.info.height = self.height
        self.map.info.origin.position.x = ((-self.height*self.resolution)/2) + (self.resolution/2)
        self.map.info.origin.position.y = ((-self.width*self.resolution)/2) + (self.resolution/2)
        self.map.info.origin.position.z = 0
        self.map.info.origin.orientation.x = 0.0
        self.map.info.origin.orientation.y = 0.0
        self.map.info.origin.orientation.z = 0.0
        self.map.info.origin.orientation.w = 0.1

        # restore map to initial map before adding obstacles (so you remove old ones)
        self.restore_map()
        print(len(self.obs_grid))
        for index in self.obs_grid:
            self.map.data[index] = 100
        
        self.map_pub.publish(self.map)
        self.count_id += 1

def main():
    rospy.init_node('MapGenerator_node_py', anonymous=True)
    mg = MapGenerator()
    rospy.spin()

if __name__ == '__main__':
    main()

