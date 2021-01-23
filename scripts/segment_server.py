#!/usr/bin/env python

import os
import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2
from point_cloud_segmentation.srv import CurrentSegment, SegmentScene, PointCloudIO


# class PointCloudSegmentation:
#     def __init__(self):
#         self.pc = PointCloud2()
    
#     def callback(self):
#         rospy.init_node('segment_server_node', anonymous=True)
#         rospy.Service('segment_server', CurrentSegment, self.handle_pointcloud)
#         rospy.spin()
    
#     def pointcloud_callback(self, pointcloud_info):
#         self.pc = pointcloud_info
    
#     def handle_pointcloud(self, req):
#         assert (req.input_str == 'pointcloud')

#         pc_sub = rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, self.pointcloud_callback, queue_size=1)

#         rospy.wait_for_service('/point_cloud_segmenation/point_cloud_segmentation/scene_segmentation')
#         segment_func = rospy.ServiceProxy('/point_cloud_segmenation/point_cloud_segmentation/scene_segmentation', SegmentScene)
#         _segmentation = segment_func(self.pc)

#         return _segmentation.segmented_scene


class CachePointCloud:
    def __init__(self):
        self.pc = PointCloud2()


current_pointcloud = CachePointCloud()


def pointcloud_callback(pointcloud_info):
    current_pointcloud.pc = pointcloud_info


def handle_pointcloud(req):
    assert (req.input_str == 'pointcloud')
    # rospy.wait_for_service('/point_cloud_segmenation/point_cloud_segmentation/scene_segmentation')
    segment_func = rospy.ServiceProxy('/point_cloud_segmenation/point_cloud_segmentation/scene_segmentation', SegmentScene)
    
    rospy.wait_for_service('pcl_dump')
    pcl_dump_func = rospy.ServiceProxy('pcl_dump', PointCloudIO)

    data_directory = "/home/ydkz/catkin_lyu/src/point_cloud_segmentation/data/scene_" 

    file_ending = ".ply"
    file_path = data_directory + "colored" + file_ending
    pcl_dump_func(file_ending, file_path, current_pointcloud.pc)

    
    print('original width %i, height %i'%(current_pointcloud.pc.width, current_pointcloud.pc.height) )
    _segmentation = segment_func(current_pointcloud.pc)
    colored_pc = _segmentation.segmented_scene.colored_cloud
    print('segmented width %i, height %i'%(colored_pc.width, colored_pc.height) )

    return _segmentation.segmented_scene


def server():
    rospy.init_node('segment_server_node', anonymous=True)
    # rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, pointcloud_callback, queue_size=1)
    rospy.Subscriber('/camera/depth/color/points', PointCloud2, pointcloud_callback, queue_size=10)
    rospy.wait_for_service('/point_cloud_segmenation/point_cloud_segmentation/scene_segmentation')
    rospy.Service('segment_server', CurrentSegment, handle_pointcloud)
    rospy.loginfo("Ready to segment pointcloud.")
    rospy.spin()


if __name__ == "__main__":
    server()
    # pcs = PointCloudSegmentation()
    # pcs.callback()
    

