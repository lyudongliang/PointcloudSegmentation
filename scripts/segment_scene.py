#!/usr/bin/env python

import os
import rospy
import time
import numpy as np
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from pointcloud_segmentation.srv import SegmentScene
from sklearn.neighbors import NearestNeighbors


MAX_DIST = 1000
VECTOR_EPSILON = 0.0001
RADIUS_EPSILON = 0.02
PT_NUM = 20
LIMITED_PT_NUM = 50
PT_COLORS = [[255, 0, 0], [0, 255, 0], [0, 0, 255], [0, 255, 255], [255, 0, 255], [255, 255, 0], [128, 0, 0], [0, 128, 0], [0, 0, 128]]


def get_cloud_pts(cloud_info):
    uv_list = [[i, j] for i in range(cloud_info.width) for j in range(cloud_info.height)]
    gen_3d = pc2.read_points(cloud_info, skip_nans=True, field_names=('x', 'y', 'z'), uvs=uv_list)

    pts_3d_list = []
    for p in gen_3d:
        if  -MAX_DIST < p[0] < MAX_DIST and -MAX_DIST < p[1] < MAX_DIST and -MAX_DIST < p[2] < MAX_DIST:
            pts_3d_list.append((p[0], p[1], p[2]))

    pts_3d_list = list(filter(lambda pt:  0.0 <  pt[2] < 3.0, pts_3d_list))
    print('len of cloud pts', len(pts_3d_list))
    return pts_3d_list


def dump_obj_file(file_path, pt_list):
    with open(file_path, 'w') as f:
        for pt in pt_list:
            pt_str = list(map(str, pt))
            line = 'v ' +  ' '.join(pt_str) + ' \n'
            f.write(line)


def get_pt_flatness_unit_vt(pt, neigh_list):
    if len(neigh_list) < LIMITED_PT_NUM:
        limited_neigh_pts = neigh_list
    else:
        new_indices = np.random.permutation(np.arange(len(neigh_list)))
        limited_neigh_pts = [neigh_list[new_indices[i]] for i in range(LIMITED_PT_NUM)]
    
    limited_neigh_pts = neigh_list
    sum_unit_vt = np.array([0., 0., 0.])
    vt_count = 0
    for neigh_pt in limited_neigh_pts:
        tangent_vt = neigh_pt - pt
        tangent_vt_norm = np.linalg.norm(tangent_vt)
        if tangent_vt_norm > VECTOR_EPSILON:
            tangent_vt /= tangent_vt_norm
            sum_unit_vt += tangent_vt
            vt_count += 1
    
    if vt_count == 0:
        return 0

    average_vt = sum_unit_vt / vt_count
    return int(np.linalg.norm(average_vt) < 0.1)


def get_pt_flatness(pt, neigh_list):
    if len(neigh_list) == 0:
        return 0
    
    mean_vt = np.mean(np.array(neigh_list) - np.array(pt), axis=0)
    return int(np.linalg.norm(mean_vt) < 0.001)


def calc_labels(pts):
    import time
    start_time = time.time()
    print('pts shape', pts.shape)
    
    neigh = NearestNeighbors(radius=RADIUS_EPSILON)

    neigh.fit(pts)  
    
    neigh_matrix = neigh.radius_neighbors_graph(pts)
    indices = neigh_matrix.indices
    indptr = neigh_matrix.indptr

    # get kernel pts.
    core_pts = list()
    core_pt_indices = list()
    for i in range(len(indptr) - 1):
        col_indices = indices[indptr[i]:indptr[i + 1]]
        if len(col_indices) > PT_NUM:
            core_pt_indices.append(i)
            core_pts.append(list(pts[i]))

    print('core pts count', len(core_pts))
    print('get neighbor time', time.time() - start_time)

    obj_file = os.path.join(os.path.abspath(''), 'src/pointcloud_segmentation/data/kernel_scene.obj')
    dump_obj_file(obj_file, core_pts)

    # calc flatness for core pts.
    pt_flatness = np.array([get_pt_flatness(pts[i], pts[indices[indptr[i]:indptr[i + 1]]])  for i in range(len(indptr) - 1)])
    print('pt flatness', pt_flatness)
    print('flat count', np.sum(pt_flatness))

    pt_labels = np.full((len(pts), ), -1)

    def _get_neighbor(pt_index, check_label):
        _neighbor_list = list()
        candidate_indices = indices[indptr[pt_index]:indptr[pt_index + 1]]
        for _index in candidate_indices:
            if pt_flatness[_index] == 1 and (check_label and pt_labels[_index] == -1 or not check_label):
                _neighbor_list.append(_index)
        
        return _neighbor_list
    
    cluster_id = 0
    for pt_id in core_pt_indices:
        if pt_labels[pt_id] == -1:
            pt_labels[pt_id] = cluster_id
            pt_neighbors = _get_neighbor(pt_id, check_label=True)
            seeds = set(pt_neighbors)

            while len(seeds) > 0:
                new_pt_id = seeds.pop()
                pt_labels[new_pt_id] = cluster_id
                query_indices = _get_neighbor(new_pt_id, check_label=False)
                if len(query_indices) >= PT_NUM:
                    for query_id in query_indices:
                        if pt_labels[query_id] == -1:
                            seeds.add(query_id)
            cluster_id += 1

    print('labels len', len(set(pt_labels)))
    label_dict = {label_index: [] for label_index in set(pt_labels)}
    for i, _label in enumerate(pt_labels):
        label_dict[_label].append(i)
    
    return label_dict


def dump_label(file_path, label_dict):
    with open(file_path, 'w') as f:
        for _label, _indices in label_dict.items():
            line = str(_label) + ' ' + str(len(_indices)) + ' ' + ' '.join(list(map(str, _indices))) + '\n'
            f.write(line)


def get_colored_pts(pt_labels_dict, pt_list):
    pt_labels_list = [[_label, _indices] for _label, _indices in pt_labels_dict.items()]
    pt_labels_list = list(sorted(pt_labels_list, key=lambda line: -len(line[1])))

    _colored_pts = []
    for _new_cluster_id, line in enumerate(pt_labels_list):
        _label, _indices = line
        if _label == -1 or _new_cluster_id >= len(PT_COLORS):
            continue
        for _pt_id in _indices:
            pt = pt_list[_pt_id]
            pt_color = PT_COLORS[_new_cluster_id]
            _colored_pts.append([pt[0], pt[1], pt[2], pt_color[0], pt_color[1], pt_color[2]])
    
    return _colored_pts


def handle_scene(req):
    print('handle scene------------------')
    print('cloud width: %i, cloud height: %i'%(req.cloud_in.width, req.cloud_in.height))

    TIME_STR = time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime())

    cloud_pts = get_cloud_pts(req.cloud_in)
    obj_file = os.path.join(os.path.abspath(''), 'src/pointcloud_segmentation/data', TIME_STR + '_original_scene.obj')
    dump_obj_file(obj_file, cloud_pts)

    cloud_pt_labels = calc_labels(np.array(cloud_pts, dtype=np.float32))
    # label_file = os.path.join(os.path.abspath(''), 'src/pointcloud_segmentation/data/pt_label.txt')
    # dump_label(label_file, cloud_pt_labels)

    segmented_obj_file = os.path.join(os.path.abspath(''), 'src/pointcloud_segmentation/data', TIME_STR + '_segmented_scene.obj')
    colored_pts = get_colored_pts(cloud_pt_labels, cloud_pts)
    dump_obj_file(segmented_obj_file, colored_pts)

    pass


def scene_server():
    rospy.init_node('scene_segmentation_node', anonymous=True)
    
    rospy.Service('scene_segmentation', SegmentScene, handle_scene)
    rospy.loginfo("Ready to segment scene.")
    rospy.spin()



if __name__ == '__main__':
    scene_server()