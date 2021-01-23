#!/usr/bin/env python

import os
import rospy
import copy
import numpy as np
from sensor_msgs.msg import PointCloud2
from point_cloud_segmentation.srv import CurrentSegment, SegmentScene, PointCloudIO


class CachePointCloud:
    def __init__(self):
        self.pc = PointCloud2()


current_pointcloud = CachePointCloud()


# def pointcloud_callback(pointcloud_info):
#     # rospy.loginfo('xxxx')
#     current_pointcloud.pc = copy.deepcopy(pointcloud_info)
#     print(current_pointcloud.pc.width, current_pointcloud.pc.height)
#     # print('pointcloud_info', pointcloud_info)


# def client():
#     print('start')
#     rospy.init_node('segment_client_node', anonymous=True)
#     rospy.Subscriber('/zed2/zed_node/point_cloud/cloud_registered', PointCloud2, pointcloud_callback, queue_size=1)

#     rospy.wait_for_service('/point_cloud_segmenation/point_cloud_segmentation/scene_segmentation')
#     segment_func = rospy.ServiceProxy('/point_cloud_segmenation/point_cloud_segmentation/scene_segmentation', SegmentScene)
#     cur_segmentation = segment_func(current_pointcloud.pc)

#     # print('current_pointcloud.pc', current_pointcloud.pc)

#     colored_pc = cur_segmentation.segmented_scene.colored_cloud
#     # print('colored_pc', colored_pc)
#     print('colored_pc type', type(colored_pc))

#     segmented_pc_list = cur_segmentation.segmented_scene.segmented_clouds

#     print('segmented_pc_list type', type(segmented_pc_list))
#     print('segmented_pc_list len', len(segmented_pc_list))

#     print('indices type', type(cur_segmentation.segmented_scene.Indices))
#     print('indices len', len(cur_segmentation.segmented_scene.Indices))

#     rospy.wait_for_service('pcl_dump')
#     pcl_dump_func = rospy.ServiceProxy('pcl_dump', PointCloudIO)

#     data_directory = "/home/ydkz/catkin_lyu/src/point_cloud_segmentation/data/scene_" 

#     file_ending = ".ply"
#     file_path = data_directory + "colored" + file_ending
#     pcl_dump_func(file_ending, file_path, colored_pc)

#     for i, segment_pc in enumerate(segmented_pc_list):
#         file_path = data_directory + str(i) +file_ending
    
#         pcl_dump_func(file_ending, file_path, segment_pc)


def client(seg_str='pointcloud'):
    rospy.wait_for_service('segment_server')
    
    seg_func = rospy.ServiceProxy('segment_server', CurrentSegment)
    cur_segmentation = seg_func(seg_str)

    colored_pc = cur_segmentation.segmented_scene.colored_cloud
    print('colored_pc type', type(colored_pc))

    segmented_pc_list = cur_segmentation.segmented_scene.segmented_clouds

    print('segmented_pc_list type', type(segmented_pc_list))
    print('segmented_pc_list len', len(segmented_pc_list))

    print('indices type', type(cur_segmentation.segmented_scene.Indices))
    print('indices len', len(cur_segmentation.segmented_scene.Indices))

    rospy.wait_for_service('pcl_dump')
    pcl_dump_func = rospy.ServiceProxy('pcl_dump', PointCloudIO)

    data_directory = "/home/ydkz/catkin_lyu/src/point_cloud_segmentation/data/scene_" 

    file_ending = ".ply"
    file_path = data_directory + "colored" + file_ending
    pcl_dump_func(file_ending, file_path, colored_pc)

    for i, segment_pc in enumerate(segmented_pc_list):
        file_path = data_directory + str(i) +file_ending
        
        dump_res = pcl_dump_func(file_ending, file_path, segment_pc)
        # print('dump status', dump_res.status)
    
    # rospy.spin()



if __name__ == "__main__":
    client()

