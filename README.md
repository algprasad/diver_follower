# diver_follower
A ROS package for diver following with an underwater robot that detects the diver using yolov3 implemented using darkent_ros

Installation instructions
The following packages are needed: 
1) startup_node #the hippocampus startup node
2) pid_control
3) diver_mover
4) darknet_ros_alg

#Running the demo diver follower
 roslaunch startup_node hippocampus_uuv_world_pid_control_stereo_camera.launch
 roslaunch diver_follower diver_following.launch 
 rosrun diver_mover diver_mover
