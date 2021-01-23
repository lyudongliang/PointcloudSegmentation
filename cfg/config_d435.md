# use d435
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

roslaunch point_cloud_segmentation segmentation_d435.launch segmentation_algorithm:="region"
roslaunch point_cloud_segmentation segmentation_d435.launch segmentation_algorithm:="region" smoothness_threshold:=0.15 curvature_threshold:=0.015

rosrun point_cloud_segmentation segment_server.py

rosrun point_cloud_segmentation save_pointcloud

rosrun point_cloud_segmentation segment_client.py
