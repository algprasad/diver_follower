/**
ROS Node to estimtae the diver's pose in global frame using diver detection bounding box, depth points.

 */

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include "Utils.h"
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include "TrackingFilterbackup.h"



//global variable so it can publish from inside the callback
ros::Publisher pub_diver_pose;
//TODO: Make these two functions into one
geometry_msgs::PoseStamped measured_diver_global_pose;

bool new_measurement = false;

Eigen::Matrix4d covertPose2HomogenousCoordinates(geometry_msgs::PoseStamped current_robot_pose){
    Eigen::Matrix4d pose4d;
    Eigen::Quaterniond robot_orientation_quat;
    robot_orientation_quat.x() = current_robot_pose.pose.orientation.x;
    robot_orientation_quat.y() = current_robot_pose.pose.orientation.y;
    robot_orientation_quat.z() = current_robot_pose.pose.orientation.z;
    robot_orientation_quat.w() = current_robot_pose.pose.orientation.w;

    Eigen::Matrix3d robot_orientation_3d;
    robot_orientation_3d = robot_orientation_quat;

    pose4d << robot_orientation_3d(0,0), robot_orientation_3d(0,1), robot_orientation_3d(0,2), current_robot_pose.pose.position.x,
            robot_orientation_3d(1,0), robot_orientation_3d(1,1), robot_orientation_3d(1,2), current_robot_pose.pose.position.y,
            robot_orientation_3d(2,0), robot_orientation_3d(2,1), robot_orientation_3d(2,2), current_robot_pose.pose.position.z,
            0,                                  0,                                     0,                           1;
    return pose4d;
}


geometry_msgs::PoseStamped getDiverGlobalPoseUtil(geometry_msgs::PoseStamped current_robot_pose, double diver_x_wrtc, double diver_y_wrtc, double diver_z_wrtc){

    //convert robot pose quaternion to rotation matrix
    Eigen::Matrix4d robot_pose_4d = covertPose2HomogenousCoordinates(current_robot_pose);


    //std::cout<<"Robot Pose 4d: "<<robot_pose_4d<<std::endl;
    Eigen::Matrix4d camera_opt_wrt_baselink_4d;
    ///Fixed for the hiipocampus model. Maybe use the .yaml file
    camera_opt_wrt_baselink_4d << 0, 0, 1, 0.2,
            -1, 0, 0, 0,
            0, -1, 0, 0.03,
            0, 0, 0, 1;

    Eigen::Matrix4d diver_wrt_camera_opt_4d;
    diver_wrt_camera_opt_4d <<  1, 0, 0, diver_x_wrtc,
            0, 1, 0, diver_y_wrtc,
            0, 0, 1, diver_z_wrtc,
            0, 0, 0, 1;

    Eigen::Matrix4d diver_global_pose_4d;
    diver_global_pose_4d = robot_pose_4d*camera_opt_wrt_baselink_4d*diver_wrt_camera_opt_4d;

    measured_diver_global_pose.pose.position.x = diver_global_pose_4d(0, 3);
    measured_diver_global_pose.pose.position.y = diver_global_pose_4d(1, 3);
    measured_diver_global_pose.pose.position.z = diver_global_pose_4d(2, 3);
    //add orientation
    measured_diver_global_pose.pose.orientation.x = 0;
    measured_diver_global_pose.pose.orientation.y = 0;
    measured_diver_global_pose.pose.orientation.z = 0;
    measured_diver_global_pose.pose.orientation.w = 1;

    return measured_diver_global_pose;

}

geometry_msgs::PoseStamped getDiverGlobalPose(darknet_ros_msgs::BoundingBox bounding_box, sensor_msgs::PointCloud2 point_cloud, geometry_msgs::PoseStamped drone_pose){

    //get the smaller bounding box with x% and y% shaved off
    int x_min, y_min, x_max, y_max;
    x_min = shaveOffMin(20, bounding_box.xmin, bounding_box.xmax);
    x_max = shaveOffMax(20, bounding_box.xmin, bounding_box.xmax);
    y_min = shaveOffMin(30, bounding_box.ymin, bounding_box.ymax);
    y_max = shaveOffMax(30, bounding_box.ymin, bounding_box.ymax);

    std::vector<geometry_msgs::Point> points3d_inside_box;

    //get 3d values of all points inside the shaved off bounding box
    int count_total_points = 0;
    int count_valid_points = 0;
    double sum_x = 0, sum_y = 0, sum_z = 0;
    for(int u = x_min; u < x_max; u++){
        for(int v = y_min; v < y_max; v++){
            geometry_msgs::Point temp_point;
            pixelTo3DPoint(point_cloud, u, v, temp_point);
            //std::cout<<temp_point<<std::endl;
            count_total_points++;
            if(!std::isnan(temp_point.z) && !std::isnan(temp_point.x) && !std::isnan(temp_point.y)){
                sum_x+= temp_point.x;
                sum_y+= temp_point.y;
                sum_z+= temp_point.z;
                count_valid_points++;
            }
        }

    }
    double average_x = sum_x/count_valid_points;
    double average_y = sum_y/count_valid_points;
    double average_z = sum_z/count_valid_points;
    return getDiverGlobalPoseUtil(drone_pose, average_x, average_y, average_z);


}

void diverTrackingCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_boxes_msg,
                           const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg,
                           const geometry_msgs::PoseStampedConstPtr& drone_pose_msg
                                                   ) {

    darknet_ros_msgs::BoundingBox bbox_main =  bounding_boxes_msg->bounding_boxes[0];
    const sensor_msgs::PointCloud2 main_point_cloud = *point_cloud_msg;
    const geometry_msgs::PoseStamped drone_pose = *drone_pose_msg;

    //get measured diver's pose from points and bounding box //this actually gets the diver's pose in global frame
    geometry_msgs::PoseStamped diver_global_pose = getDiverGlobalPose(bbox_main, main_point_cloud, drone_pose);
    new_measurement = true;

}



int main(int argc, char** argv){
    ros::init(argc, argv, "diver_tracking_filter");
    ros::NodeHandle nh;
    pub_diver_pose = nh.advertise<geometry_msgs::PoseStamped>("hippocampus/diver_follower/diver_pose_filtered", 10);


    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bounding_box(nh, "/darknet_ros/bounding_boxes", 30);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_depth_points(nh, "/stereo/points2", 30 );
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_drone_pose(nh, "/mavros/local_position/pose", 30);

    typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2, geometry_msgs::PoseStamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync2(MySyncPolicy(30), sub_bounding_box, sub_depth_points, sub_drone_pose);
    sync2.registerCallback(&diverTrackingCallback);



    //TODO: source of error: Change the ros frequency is this is too high
    double ros_freq = 20;
    ros::Rate rate(ros_freq);
    TrackingFilter tracking_filter(ros_freq);

    //TODO: get the initial values of the state
    while(!tracking_filter.initialized_state){
        if(new_measurement){
            tracking_filter.initializeState(measured_diver_global_pose);
        }

        ros::spinOnce();
    }
    ROS_INFO("State Initialized!");
    geometry_msgs::PoseStamped diver_filtered_pose;

    while(ros::ok()){
        ROS_INFO("Getting filter estimates");


        //tracking_filter
        tracking_filter.cycle(new_measurement, measured_diver_global_pose, diver_filtered_pose); //the stamp is assigned inside the function itself


        pub_diver_pose.publish(diver_filtered_pose);
        new_measurement = false;
        ros::spinOnce();
        //rate.sleep();
    }

    return 0;

}
