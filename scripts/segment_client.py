#!/usr/bin/env python

import os
import rospy
import copy
import numpy as np
from sensor_msgs.msg import PointCloud2
from pointcloud_segmentation.srv import SegmentStatus, SegmentScene, PointCloudIO


def client(seg_str='pointcloud'):
    rospy.wait_for_service('trigger_segmentation')
    
    trigger_seg_func = rospy.ServiceProxy('trigger_segmentation', SegmentStatus)
    cur_segmentation = trigger_seg_func(seg_str)
    # print


if __name__ == "__main__":
    client()

