//
// Created by alg on 23/02/21.
//

#include <iostream>
#include "TrackingFilterbackup.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>


bool new_measurement = false;
geometry_msgs::PoseStamped measured_diver_global_pose;

void measuredPoseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
    measured_diver_global_pose = *msg;
    new_measurement = true;
}
int main(int argc, char** argv){
    ros::init(argc, argv, "diver_filtered_pose");
    ros::NodeHandle nh;
    ros::Subscriber sub_measured_pose = nh.subscribe<geometry_msgs::PoseStamped>("hippocampus/diver_follower/diver_pose_measurement", 10, measuredPoseCallback);
    ros::Publisher pub_diver_pose =nh.advertise<geometry_msgs::PoseStamped>("hippocampus/diver_follower/diver_pose_filtered", 10);
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
   /* //TODO: REMOVE THIS NON SENSE OF INITIALIZING THE DIVER POSE DIRECTLY
    geometry_msgs::PoseStamped initial_pose;
    initial_pose.pose.position.x = 2.5;
    initial_pose.pose.position.y = 0;
    initial_pose.pose.position.z = -2;

    tracking_filter.initializeState(initial_pose);*/
    ROS_INFO("State Initialized!");
    geometry_msgs::PoseStamped diver_filtered_pose;

    while(ros::ok()){
        ROS_INFO("Getting filter estimates");

        //tracking_filter
        tracking_filter.cycle(new_measurement, measured_diver_global_pose, diver_filtered_pose); //the stamp is assigned inside the function itself


        pub_diver_pose.publish(diver_filtered_pose);
        new_measurement = false;
        ros::spinOnce();
    }

    return 0;
}