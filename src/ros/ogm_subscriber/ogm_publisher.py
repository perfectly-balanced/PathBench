import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from std_msgs.msg import *

import cv2 as cv
import numpy as np

import os

def load_grid():
    fn = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'ogm.png')
    img = cv.imread(fn, 0)
    height, width = img.shape
    data = np.array([item for sublist in img for item in sublist]).astype(np.int8).tolist()

    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    grid.data = data
    grid.info.resolution = 1
    grid.info.width = width
    grid.info.height = height
    grid.info.origin.position.x = 0.0
    grid.info.origin.position.y = 0.0
    grid.info.origin.position.z = 0.0
    grid.info.origin.orientation.w = 1.0
    grid.info.resolution = 1
    return grid

def main():
    rospy.init_node('map', anonymous=True)
    pub = rospy.Publisher('/map', OccupancyGrid, latch=True, queue_size=1)

    pub.publish(load_grid())
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
