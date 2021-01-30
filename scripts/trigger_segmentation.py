#!/usr/bin/env python

import os
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from pointcloud_segmentation.srv import SegmentStatus, SegmentScene, PointCloudIO


class CachePointCloud:
    def __init__(self):
        self.pc = PointCloud2()


current_pointcloud = CachePointCloud()


def pointcloud_callback(pointcloud_info):
    current_pointcloud.pc = pointcloud_info


def handle_pointcloud(req):
    assert (req.input_str == 'pointcloud')
    segment_func = rospy.ServiceProxy('scene_segmentation', SegmentScene)
    
    rospy.wait_for_service('dump_scene_service')
    pcl_dump_func = rospy.ServiceProxy('dump_scene_service', PointCloudIO)

    data_directory = "/home/ydkz/catkin_lyu/src/point_cloud_segmentation/data/scene_" 

    file_ending = ".ply"
    file_path = data_directory + "colored" + file_ending
    pcl_dump_func(file_ending, file_path, current_pointcloud.pc)

    
    print('original width %i, height %i'%(current_pointcloud.pc.width, current_pointcloud.pc.height) )
    _segmentation = segment_func(current_pointcloud.pc)

    return _segmentation.segment_status


def server():
    rospy.init_node('trigger_segmentation_node', anonymous=True)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, pointcloud_callback, queue_size=10)
    rospy.wait_for_service('scene_segmentation')
    rospy.Service('trigger_segmentation', SegmentStatus, handle_pointcloud)
    rospy.loginfo("Ready to trigger segmentation.")
    rospy.spin()


if __name__ == "__main__":
    server()
    
