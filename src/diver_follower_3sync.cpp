//
// Created by alg on 21/09/20.
//
#include <iostream>
#include <ros/ros.h>
#include <stereo_msgs/DisparityImage.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <geometry_msgs/PoseStamped.h>
#include "eigen3/Eigen/Geometry"
#include "Utils.h"

static int seq = 1;

ros::Publisher pub_target_drone_pose;
geometry_msgs::PoseStamped setDroneTargetPose(geometry_msgs::PoseStamped diver_global_pose, geometry_msgs::PoseStamped robot_global_pose){
    Eigen::Vector3d diver_global_position, robot_global_position;
    diver_global_position << diver_global_pose.pose.position.x , diver_global_pose.pose.position.y, diver_global_pose.pose.position.z;
    robot_global_position << robot_global_pose.pose.position.x, robot_global_pose.pose.position.y, robot_global_pose.pose.position.z;
    Eigen::Vector3d distance_vector;

    distance_vector = diver_global_position - robot_global_position;
    Eigen::Vector3d unit_distance_vector;
    double distance_vector_norm = std::sqrt((distance_vector(0)*distance_vector(0)) +
                                            (distance_vector(1)*distance_vector(1))+
                                            (distance_vector(2)*distance_vector(2)));
    unit_distance_vector << distance_vector(0)/distance_vector_norm,
            distance_vector(1)/distance_vector_norm,
            distance_vector(2)/distance_vector_norm;

    //TODO Should get this from .yaml file
    //distance to maintain from the diver
    double d2m = 3;
    Eigen::Vector3d d2m_vector;
    d2m_vector <<  -d2m*unit_distance_vector(0),
            -d2m*unit_distance_vector(1),
            -d2m*unit_distance_vector(2);

    Eigen::Vector3d position_setpoint;
    position_setpoint << diver_global_position(0) + d2m_vector(0),
            diver_global_position(1) + d2m_vector(1),
            diver_global_position(2);

    geometry_msgs::PoseStamped target_robot_pose;
    target_robot_pose.pose.position.x = position_setpoint(0);
    target_robot_pose.pose.position.y = position_setpoint(1);
    target_robot_pose.pose.position.z = position_setpoint(2);
    return target_robot_pose;
}

geometry_msgs::PoseStamped calculateDiverGlobalPose(geometry_msgs::PoseStamped current_robot_pose, double diver_x_wrtc, double diver_y_wrtc, double diver_z_wrtc){

    //convert robot pose quaternion to rotation matrix
    Eigen::Matrix4d robot_pose_4d;
    Eigen::Quaterniond robot_orientation_quat;
    robot_orientation_quat.x() = current_robot_pose.pose.orientation.x;
    robot_orientation_quat.y() = current_robot_pose.pose.orientation.y;
    robot_orientation_quat.z() = current_robot_pose.pose.orientation.z;
    robot_orientation_quat.w() = current_robot_pose.pose.orientation.w;

    Eigen::Matrix3d robot_orientation_3d;
    robot_orientation_3d = robot_orientation_quat;

    robot_pose_4d << robot_orientation_3d(0,0), robot_orientation_3d(0,1), robot_orientation_3d(0,2), current_robot_pose.pose.position.x,
            robot_orientation_3d(1,0), robot_orientation_3d(1,1), robot_orientation_3d(1,2), current_robot_pose.pose.position.y,
            robot_orientation_3d(2,0), robot_orientation_3d(2,1), robot_orientation_3d(2,2), current_robot_pose.pose.position.z,
            0,                                  0,                                     0,                           1;

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
    geometry_msgs::PoseStamped diver_global_pose;
    diver_global_pose.pose.position.x = diver_global_pose_4d(0,3);
    diver_global_pose.pose.position.y = diver_global_pose_4d(1,3);
    diver_global_pose.pose.position.z = diver_global_pose_4d(2,3);
    //add orientation
    diver_global_pose.pose.orientation.x = 0;
    diver_global_pose.pose.orientation.y = 0;
    diver_global_pose.pose.orientation.z = 0;
    diver_global_pose.pose.orientation.w = 1;

    return diver_global_pose;

}

geometry_msgs::PoseStamped getDiverPosewrtCamera(darknet_ros_msgs::BoundingBox bounding_box, sensor_msgs::PointCloud2 point_cloud, geometry_msgs::PoseStamped drone_pose){

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
            if(!isnan(temp_point.z) && !isnan(temp_point.x) && !isnan(temp_point.y)){
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
    std::cout<<"Average Point: "<<average_x<<" "<<average_y<<" "<<average_z<<std::endl;
    return calculateDiverGlobalPose(drone_pose, average_x, average_y, average_z);


}
void boundingBoxDepthImageDronePoseCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_boxes_msg,
                                            const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg,
                                            const geometry_msgs::PoseStampedConstPtr& drone_pose_msg){

    ///get all the messages
    darknet_ros_msgs::BoundingBox bbox_main =  bounding_boxes_msg->bounding_boxes[0];
    const sensor_msgs::PointCloud2 main_point_cloud = *point_cloud_msg;
    const geometry_msgs::PoseStamped drone_pose = *drone_pose_msg;

    //get the diver pose wrt camera
    geometry_msgs::PoseStamped diver_global_pose = getDiverPosewrtCamera(bbox_main, main_point_cloud, drone_pose);
    //std::cout<<diver_global_pose;

    //calculate drone target pose
    geometry_msgs::PoseStamped drone_target_pose_msg = setDroneTargetPose(diver_global_pose, drone_pose);



    //publish all TFs for rviz visualization
    publishTf(drone_pose, "robot");
    publishTf(diver_global_pose, "diver");

    //publish drone target pose
    drone_target_pose_msg.header.seq = seq++;
    std::cout<<drone_target_pose_msg<<std::endl;
    pub_target_drone_pose.publish(drone_target_pose_msg);


}



int main(int argc, char** argv){
    ros::init(argc, argv, "diver_follower_sync");
    ros::NodeHandle nh;
    pub_target_drone_pose = nh.advertise<geometry_msgs::PoseStamped>("hippocampus/desired_pose", 10);


    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bounding_box(nh, "/darknet_ros/bounding_boxes", 30);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_depth_points(nh, "/stereo/points2", 30 );
    message_filters::Subscriber<geometry_msgs::PoseStamped> sub_drone_pose(nh, "/mavros/local_position/pose", 30);

    typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2, geometry_msgs::PoseStamped > MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync2(MySyncPolicy(30), sub_bounding_box, sub_depth_points, sub_drone_pose);
    sync2.registerCallback(&boundingBoxDepthImageDronePoseCallback);
    ros::spin();

}
