#!/usr/bin/env python

import os
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from point_cloud_segmentation.srv import SegmentScene


def handle_scene(req):
    print('handle scene.')
    print('cloud width: %i, cloud height: %i'%(req.cloud_in.width, req.cloud_in.height))

    pass


def scene_server():
    rospy.init_node('scene_segmentation_node', anonymous=True)
    
    rospy.Service('scene_segmentation', SegmentScene, handle_scene)
    rospy.loginfo("Ready to segment scene.")
    rospy.spin()



if __name__ == '__main__':
    scene_server()