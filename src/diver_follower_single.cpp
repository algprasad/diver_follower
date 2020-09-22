//
// Created by alg on 10/09/20.
//

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include "eigen3/Eigen/Geometry"
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/core/core.hpp>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace message_filters;

struct Pixel {
    long u;
    long v;
    Pixel(): u(0), v(0){}
};

geometry_msgs::PoseStamped target_robot_pose;
geometry_msgs::PoseStamped current_robot_pose;

void callback(const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_box,
              const sensor_msgs::PointCloud2ConstPtr& depth_points)
{
    darknet_ros_msgs::BoundingBox box_left =  bounding_box->bounding_boxes[0];

    //get the list of all points
    double x_mid = (box_left.xmax + box_left.xmin)/2;
    double y_mid = (box_left.ymax + box_left.ymin)/2;
    std::cout<<x_mid<<"xx"<< y_mid<<std::endl;

    //TODO get this value from the yaml file
    double image_height = 400;
    double image_width  =  400;

    std::cout<<"Dimesnions of points cloud " <<depth_points->height<<" X "<<depth_points->width<<std::endl;
    //std::cout<<"Mid Depth "<<depth_points->data[x_mid];

    for(int i =0; i<image_height; i++){
        int indx = x_mid*199 + i;
        std::cout<<"Depth "<<depth_points->data[indx];

    }
  /*






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
            0, 0, 0, 1;
    Eigen::Matrix4d camera_opt_wrt_baselink_4d;
    ///Fixed for the hiipocampus model. Maybe use the .yaml file
    camera_opt_wrt_baselink_4d << 0, 0, 1, 0.2,
            0, 1, 0, 0,
            -1, 0, 0, 0.03,
            0, 0, 0, 1;

    Eigen::Matrix4d diver_wrt_camera_opt_4d;
    diver_wrt_camera_opt_4d << 1, 0, 0, X,
            0, 1, 0, Y,
            0, 0, 1, Z,
            0, 0, 0, 1;

    Eigen::Matrix4d diver_global_pose_4d;
    diver_global_pose_4d = robot_pose_4d*camera_opt_wrt_baselink_4d*diver_wrt_camera_opt_4d;


    Eigen::Vector3d diver_global_position, robot_global_position;
    diver_global_position << diver_global_pose_4d(0, 3), diver_global_pose_4d(1,3), diver_global_pose_4d(2,3);
    robot_global_position << robot_pose_4d(0,3), robot_pose_4d(1,3), robot_pose_4d(2,3);
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
    d2m_vector <<   d2m*unit_distance_vector(0),
            d2m*unit_distance_vector(1),
            d2m*unit_distance_vector(2);

    Eigen::Vector3d position_setpoint;
    position_setpoint << diver_global_position(0) + d2m_vector(0),
            diver_global_position(1) + d2m_vector(1),
            diver_global_position(2);

    target_robot_pose.pose.position.x = position_setpoint(0);
    target_robot_pose.pose.position.y = position_setpoint(1);
    target_robot_pose.pose.position.z = position_setpoint(2);
    std::cout<<"TARGET POSE: "<<target_robot_pose.pose.position.x<<" "<<
             target_robot_pose.pose.position.y<<" "<<
             target_robot_pose.pose.position.z<<std::endl;*/
}

void initializeRobotPose(){
    target_robot_pose.pose.position.x = 0;
    target_robot_pose.pose.position.y = 0;
    target_robot_pose.pose.position.z = 0;



}

void mavrosGlobalPositionCallback(const geometry_msgs::PoseStampedPtr& robot_pose){
    //std::cout<<"Robot Pose Callback called"<<std::endl;
    current_robot_pose.pose.position.x = robot_pose->pose.position.x;
    current_robot_pose.pose.position.y = robot_pose->pose.position.y;
    current_robot_pose.pose.position.z = robot_pose->pose.position.z;

    current_robot_pose.pose.orientation.x = robot_pose->pose.orientation.x;
    current_robot_pose.pose.orientation.y = robot_pose->pose.orientation.y;
    current_robot_pose.pose.orientation.z = robot_pose->pose.orientation.z;
    current_robot_pose.pose.orientation.w = robot_pose->pose.orientation.w;




}

void tempDepthPointsCallback(sensor_msgs::PointCloud2ConstPtr& msg){




}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diver_follower_node");

    ros::NodeHandle nh;

    //add the publisher to publish setpoint for the drone to follow
    ros::Publisher pub_robot_target_pose = nh.advertise<geometry_msgs::PoseStamped>("hippocampus/desired_pose", 10);
    ros::Subscriber sub_current_pose = nh.subscribe("/mavros/local_position/pose", 1000, mavrosGlobalPositionCallback);
    ros::Subscriber sub_depth_temp = nh.subscribe("/stereo/points2", 10, tempDepthPointsCallback);

    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bounding_box(nh, "/darknet_ros/bounding_boxes", 100);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_depth_points(nh, "/stereo/points2", 100);

    typedef sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2> MySyncPolicy;

    //TimeSynchronizer<darknet_ros_msgs::BoundingBoxes, darknet_ros_msgs::BoundingBoxes, geometry_msgs::PoseStamped > sync(image_sub_left, image_sub_right, robot_sub,  10);
    TimeSynchronizer<darknet_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2 > sync(sub_bounding_box, sub_depth_points,  100);
    //sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    sync.registerCallback(boost::bind(&callback, _1, _2));

    initializeRobotPose();
    //ros::spin();
    ros::Rate rate(20);
    while(ros::ok()){
        //pub_robot_target_pose.publish(target_robot_pose);
        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
