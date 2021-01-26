#!/usr/bin/env python

import os
import rospy
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from pointcloud_segmentation.srv import SegmentScene
from sklearn.neighbors import NearestNeighbors


MAX_DIST = 1000


def get_pos_3d(cloud_info, x_index, y_index, obj_width, obj_height):
    uv_list = [[int(x_index+i-5), y_index+j-5] for i in range(11) for j in range(11)]

    gen_2d = pc2.read_points(cloud_info, skip_nans=True, field_names=('y', 'z'), uvs=uv_list)
    gen_depth = pc2.read_points(cloud_info, skip_nans=True, field_names=('x'), uvs=uv_list)
    
    pts_2d_list = []
    for p in gen_2d:
        if  -MAX_DIST < p[0] < MAX_DIST and -MAX_DIST < p[1] < MAX_DIST:
            pts_2d_list.append((p[0], p[1]))
    
    depth_list = []
    for p in gen_depth:
        if -MAX_DIST < p[0] < MAX_DIST:
            depth_list.append(p[0])

    if len(pts_2d_list) > 0 and len(depth_list) > 0:
        pt_2d = np.mean(np.array(pts_2d_list), 0)
        depth = np.mean(np.array(depth_list))
        pt_3d = np.array([depth, pt_2d[0], pt_2d[1]])
        return pt_3d
    else:
        return np.array([MAX_DIST, MAX_DIST, MAX_DIST])


def get_cloud_pts(cloud_info):
    uv_list = [[i, j] for i in range(cloud_info.width) for j in range(cloud_info.height)]
    gen_3d = pc2.read_points(cloud_info, skip_nans=True, field_names=('x', 'y', 'z'), uvs=uv_list)

    pts_3d_list = []
    for p in gen_3d:
        if  -MAX_DIST < p[0] < MAX_DIST and -MAX_DIST < p[1] < MAX_DIST and -MAX_DIST < p[2] < MAX_DIST:
            pts_3d_list.append((p[0], p[1], p[2]))

    pts_3d_list = list(filter(lambda pt:  0.0 <  pt[2] < 3.0, pts_3d_list))
    print('len of cloud pts', len(pts_3d_list))
    print('head 10 pts', pts_3d_list[:10])
    return pts_3d_list


def get_nearest_neighbor(pts):
    import time
    start_time = time.time()
    print('pts shape', pts.shape)
    neigh = NearestNeighbors(radius=0.01)

    neigh.fit(pts)  
    
    neigh_matrix = neigh.radius_neighbors_graph(pts)
    print(type(neigh_matrix.data))
    print(len(neigh_matrix.data))
    print(neigh_matrix.indices)
    print(len(neigh_matrix.indices))
    print(neigh_matrix.indptr)
    print(len(neigh_matrix.indptr))
    indices = neigh_matrix.indices
    indptr = neigh_matrix.indptr

    valid_count = 0
    for i in range(len(indptr) - 1):
        col_indices = indices[indptr[i]:indptr[i + 1]]
        if len(col_indices) > 20:
            # print(len(col_indices))
            valid_count += 1
    print('valid_count', valid_count)

    print('get neighbor time', time.time() - start_time)


def handle_scene(req):
    print('handle scene.')
    print('cloud width: %i, cloud height: %i'%(req.cloud_in.width, req.cloud_in.height))

    cloud_pts = get_cloud_pts(req.cloud_in)
    get_nearest_neighbor(np.array(cloud_pts, dtype=np.float32))
    pass


def scene_server():
    rospy.init_node('scene_segmentation_node', anonymous=True)
    
    rospy.Service('scene_segmentation', SegmentScene, handle_scene)
    rospy.loginfo("Ready to segment scene.")
    rospy.spin()



if __name__ == '__main__':
    scene_server()