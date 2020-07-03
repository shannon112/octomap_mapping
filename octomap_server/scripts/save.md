# save cartographer state
rosservice call /finish_trajectory "trajectory_id: 0"
rosservice call /write_state "{filename: '${HOME}/Downloads/AMIRSLAM.bag.pbstream', include_unfinished_submaps: false}"

# save octomap .ot
rosrun octomap_server octomap_saver -f AMIRSLAM.ot

# save pointcloud map .pcd
rosrun pcl_ros pointcloud_to_pcd input:=/map3d 

# save new constraint and pose array as .pkl
rosservice call /save_amir_slam_to_file "{}" 
