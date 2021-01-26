# use d435
roslaunch realsense2_camera rs_camera.launch filters:=pointcloud

rosrun pointcloud_segmentation segment_scene.py

rosrun pointcloud_segmentation trigger_segmentation.py

rosrun pointcloud_segmentation save_pointcloud

rosrun pointcloud_segmentation segment_client.py
