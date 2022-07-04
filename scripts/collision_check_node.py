"""  * Project: Object detection                    
 * Author: Giacomo Tomasi              
 * E-Mail: giacomo.tomasi@unibz.it         
 * Date: June 2022                        
 * File: collision_check_node.py                         """

from pickletools import uint8
import rospy
from std_msgs.msg import Bool, UInt16MultiArray

class CollisionCheck(object):
    def __init__(self):
        self.collision_pub = rospy.Publisher('collision', Bool, queue_size=1)
        self.obs_sub = rospy.Subscriber('/obstacle_cells', UInt16MultiArray, self.obs_callback)
        self.path_sub = rospy.Subscriber('/path_cells', UInt16MultiArray, self.path_callback)
        self.collision = Bool()
        self.collision.data = False
        self.obs = []
        self.path = []
        
    def obs_callback(self, obs_msg):
        self.obs = obs_msg.data
        # check for collision
        self.check()

    def path_callback(self, path_msg):
        self.path = path_msg.data

    def check(self):
        n = len(self.path)
        if (n == 0):
            rospy.logwarn('Path is empty. Not receiving data.')
            self.collision.data = False
        else:
            for obs_cell in self.obs:
                for path_cell in self.path:
                    if (obs_cell == path_cell):
                        self.collision.data = True
                        break
                    else:
                        self.collision.data = False
        self.collision_pub.publish(self.collision)

def main():
    pass

if __name__ == '__main__':
    main()

