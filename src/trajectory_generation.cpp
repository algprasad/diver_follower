#include <iostream>
#include "ros/ros.h"
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/GridMapRosConverter.hpp>
#include <filters/filter_chain.h>
#include <geometry_msgs/PoseStamped.h>
#include <cmath>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/String.h>
#include <math.h>
#include <nav_msgs/Path.h>
//for getting robot orientation matrix from geometry_msgs
#include "Utils.h"
#include "MeanImputation.h"

grid_map::GridMap current_grid_map;
geometry_msgs::PoseStamped robot_target_pose;
geometry_msgs::PoseStamped robot_current_pose, diver_pose;
int num_steps_diver_robot = 10;

//declaring as global variables whose values will be assingned later from the rosparam server
double distance_to_maintain = 2.5;
double grid_resolution = 0.4;
double time_between_waypoints = 0.500000000;
double zhiang_factor = 0.75; //multiplied to the z-vector NOT normal vect                                                                                         or

std::string trajectory_topic = "iris_depth_camera/command/trajectory";
bool elevation_map_recieved = false;


Eigen::Vector3d getCombinedUnitVector(Eigen::Matrix<double, 3, 1> matrix);

void printCurrentLayers(grid_map::GridMap grid_map){
    std::cout<<"Current Frame ID: "<<grid_map.getFrameId()<<std::endl;
    for (int i = 0; i< grid_map.getLayers().size(); i++){
        std::cout<<"Layers: "<<grid_map.getLayers()[i]<<std::endl;

    }

}
void printMapSize(grid_map::GridMap grid_map){
    std::cout<<"GRID MAP SIZE: "<<grid_map.getSize()(0)<<", "<<grid_map.getSize()(1)<<std::endl;
}

void elevationMapCallback(const grid_map_msgs::GridMap grid_map_msg){
    //convert the message to GridMap Type
    grid_map::GridMapRosConverter::fromMessage(grid_map_msg, current_grid_map);
    printMapSize(current_grid_map);
    //printCurrentLayers(current_grid_map);

    grid_map::Matrix current_matrix = current_grid_map.get("elevation");
    elevation_map_recieved = true;


}

void diverPoseCallback(const geometry_msgs::PoseStampedConstPtr& diver_pose_msg){
    diver_pose = *diver_pose_msg;
}

void initializePoseCommand(){
    robot_target_pose.pose.position.x = 1;
    robot_target_pose.pose.position.y = 0;
    robot_target_pose.pose.position.z = distance_to_maintain;

    robot_target_pose.pose.orientation.x = 0 ;
    robot_target_pose.pose.orientation.y = 0 ;
    robot_target_pose.pose.orientation.z = 0 ;
    robot_target_pose.pose.orientation.w = 1 ;
}

void robotPoseCallback(const geometry_msgs::PoseStamped msg){
    robot_current_pose.pose.position.x = msg.pose.position.x;
    robot_current_pose.pose.position.y = msg.pose.position.y;
    robot_current_pose.pose.position.z = msg.pose.position.z;

    robot_current_pose.pose.orientation.x = msg.pose.orientation.x;
    robot_current_pose.pose.orientation.y = msg.pose.orientation.y;
    robot_current_pose.pose.orientation.z = msg.pose.orientation.z;
    robot_current_pose.pose.orientation.w = msg.pose.orientation.w;


}

//trajectory for terrain relative navigation
void generateTrajectoryTRNbackup(nav_msgs::Path & rviz_path){
    rviz_path.poses.clear();
    Eigen::Matrix4d robot_pose4d;
    setMat4dFromPose(robot_current_pose, robot_pose4d);
    //std::cout<<"robot pose 4d: \n"<<robot_pose4d<<std::endl;

    double x_step_size = 1;
    double y_step_size = 0;


    grid_map::Matrix current_matrix = current_grid_map.get("elevation");
    fillIn(current_matrix);
    std::cout<<"Current Elevation Matrix: \n"<<current_matrix<<std::endl;

    static double default_z = 0;
    double map_centre_x, map_centre_y;
    map_centre_x = current_grid_map.getSize()(0) / 2;
    map_centre_y = current_grid_map.getSize()(1) / 2;


    int grid_index_x = map_centre_x + x_step_size/grid_resolution;
    int grid_index_y = map_centre_y;

    if(grid_index_x >= current_grid_map.getSize()[0] || grid_index_y >= current_grid_map.getSize()[1]) std::cout<<"Access exceeds grid map dimensions: grid access: "<<grid_index_x<<" "<<grid_index_y<<std::endl;
    if(std::isnan(current_matrix(grid_index_x, grid_index_y))) ROS_INFO("Nan value accessed");

    if((grid_index_x < current_grid_map.getSize()[0] || grid_index_y < current_grid_map.getSize()[1]) && (!std::isnan(current_matrix(grid_index_x, grid_index_y)))) {
        robot_target_pose.pose.position.z = current_matrix(grid_index_x, grid_index_y) + distance_to_maintain;
        default_z = robot_target_pose.pose.position.z;
    }
    else {
        robot_target_pose.pose.position.z = default_z;
        //ROS_INFO("Using default value for elevation");
    }

    //std::cout<<"robot pose: "<<robot_target_pose<<std::endl;
    robot_target_pose.pose.position.x = robot_current_pose.pose.position.x + x_step_size;
    robot_target_pose.pose.position.y = robot_current_pose.pose.position.y;



    robot_target_pose.pose.orientation.w = 1;
    robot_target_pose.pose.orientation.x = 0;
    robot_target_pose.pose.orientation.y = 0;
    robot_target_pose.pose.orientation.z = 0;


    robot_target_pose.header.stamp = ros::Time::now();


}

//trajectory for terrain relative navigation
void generateTrajectoryTRN(nav_msgs::Path & rviz_path){
    rviz_path.poses.clear();
    Eigen::Matrix4d robot_pose4d;
    setMat4dFromPose(robot_current_pose, robot_pose4d);
    //std::cout<<"robot pose 4d: \n"<<robot_pose4d<<std::endl;

    double x_step_size = 1;
    double y_step_size = 0;


    grid_map::Matrix current_matrix = current_grid_map.get("elevation");
    fillIn(current_matrix);
    std::cout<<"Current Elevation Matrix: \n"<<current_matrix<<std::endl;

    static double default_z = 0;
    double map_centre_x, map_centre_y;
    map_centre_x = current_grid_map.getSize()(0) / 2;
    map_centre_y = current_grid_map.getSize()(1) / 2;


    int grid_index_x = map_centre_x; // + x_step_size/grid_resolution;
    int grid_index_y = map_centre_y;

    if(grid_index_x >= current_grid_map.getSize()[0] || grid_index_y >= current_grid_map.getSize()[1]) std::cout<<"Access exceeds grid map dimensions: grid access: "<<grid_index_x<<" "<<grid_index_y<<std::endl;
    if(std::isnan(current_matrix(grid_index_x, grid_index_y))) ROS_INFO("Nan value accessed");

    if((grid_index_x < current_grid_map.getSize()[0] || grid_index_y < current_grid_map.getSize()[1]) && (!std::isnan(current_matrix(grid_index_x, grid_index_y)))) {
        robot_target_pose.pose.position.z = current_matrix(grid_index_x, grid_index_y) + distance_to_maintain;
        default_z = robot_target_pose.pose.position.z;
    }
    else {
        robot_target_pose.pose.position.z = default_z;
        //ROS_INFO("Using default value for elevation");
    }

    //std::cout<<"robot pose: "<<robot_target_pose<<std::endl;
    robot_target_pose.pose.position.x = robot_current_pose.pose.position.x + x_step_size;
    robot_target_pose.pose.position.y = robot_current_pose.pose.position.y;



    robot_target_pose.pose.orientation.w = 1;
    robot_target_pose.pose.orientation.x = 0;
    robot_target_pose.pose.orientation.y = 0;
    robot_target_pose.pose.orientation.z = 0;


    robot_target_pose.header.stamp = ros::Time::now();


}
void generateLocalTrajectory(nav_msgs::Path& rviz_path){
    Eigen::Matrix4d robot_pose4d, diver_pose4d;
    setMat4dFromPose(robot_current_pose, robot_pose4d);
    setMat4dFromPose(diver_pose, diver_pose4d);
    Eigen::Matrix4d rHd = robot_pose4d.inverse()*diver_pose4d; //TODO source of error: check if the orientation of diver pose is idientity.
    Eigen::Vector3d rTd;
    rTd << rHd(0, 3), rHd(1,3), rHd(2,3);
    //get the 10 waypoints in robot frame
    double x_step_size = rTd(0)/num_steps_diver_robot;
    double y_step_size = rTd(1)/num_steps_diver_robot;
    x_step_size = 1;

    grid_map::Matrix current_matrix = current_grid_map.get("elevation");
   /* grid_map::Matrix normal_x_matrix = current_grid_map.get("normal_vectors_x");
    grid_map::Matrix normal_y_matrix = current_grid_map.get("normal_vectors_y");
    grid_map::Matrix normal_z_matrix = current_grid_map.get("normal_vectors_z");*/

    double default_z = 0;
    double map_centre_x, map_centre_y;
    map_centre_x = current_grid_map.getSize()(0) / 2;
    map_centre_y = current_grid_map.getSize()(1) / 2;
    rviz_path.header.frame_id = "base_link";
    rviz_path.header.stamp = ros::Time::now();

    for(int i = 1; i<= 10; i++){
        geometry_msgs::PoseStamped temp_pose_in_robot_frame, temp_pose_in_global_frame;

        int grid_index_x = map_centre_x + x_step_size*i/grid_resolution;
        int grid_index_y = map_centre_y + y_step_size*i/grid_resolution;
        temp_pose_in_robot_frame.pose.position.x = x_step_size * i;
        temp_pose_in_robot_frame.pose.position.y = y_step_size * i;


        if((grid_index_x < current_grid_map.getSize()[0] && grid_index_y <  current_grid_map.getSize()[1]) && (!std::isnan(current_matrix(grid_index_x, grid_index_y)))) {
            temp_pose_in_robot_frame.pose.position.z = current_matrix(grid_index_x, grid_index_y) + distance_to_maintain;
            default_z = temp_pose_in_robot_frame.pose.position.z;
        }
        else temp_pose_in_robot_frame.pose.position.z = default_z;
        //std::cout<<"Pose in robot frame: "<<temp_pose_in_robot_frame<<std::endl;

        //convert the pose into global frame and add to path
        Eigen::Matrix4d desired_pose_in_robot_frame;
        setMat4dFromPose(temp_pose_in_robot_frame, desired_pose_in_robot_frame);
        Eigen::Matrix4d desired_pose_in_global_frame = robot_pose4d*desired_pose_in_robot_frame;
        temp_pose_in_global_frame.pose.position.x = desired_pose_in_global_frame(0);
        temp_pose_in_global_frame.pose.position.y = desired_pose_in_global_frame(1);
        temp_pose_in_global_frame.pose.position.z = desired_pose_in_global_frame(2);
        rviz_path.poses.push_back(temp_pose_in_global_frame);

    }
    robot_target_pose = rviz_path.poses[0];
    robot_target_pose.pose.orientation.w = 1;
    robot_target_pose.pose.orientation.x = 0;
    robot_target_pose.pose.orientation.y = 0;
    robot_target_pose.pose.orientation.z = 0;


    robot_target_pose.header.stamp = ros::Time::now();


}

void generateTrajectoryUnitMethod(){
    //get the unit vector in the direction of the diver
    Eigen::Matrix4d robot_pose4d, diver_pose4d;
    setMat4dFromPose(robot_current_pose, robot_pose4d);
    setMat4dFromPose(diver_pose, diver_pose4d);
    Eigen::Matrix4d rHd = robot_pose4d.inverse()*diver_pose4d; //TODO source of error: check if the orientation of diver pose is idientity.
    Eigen::Vector3d rTd;
    rTd << rHd(0, 3), rHd(1,3), rHd(2,3);
    std::cout<<"rTd: \n"<<rTd<<std::endl;

    //bool desired_pose_behind_robot = false;
    //if(rTd(0) < 0) desired_pose_behind_robot  = true;
    double norm = sqrt(rTd(0)*rTd(0) + rTd(1)*rTd(1) + rTd(2)*rTd(2));
    //only proceed inf norm > 0.2 .. i.e. robot is at least 20cm away from diver
    if(norm > 0.2){
        Eigen::Vector3d rTd_unit;
        //std::cout<<"rTd: \n"<<rTd<<std::endl;
        //adding 0.1 here so even if the rTd is zero it would still be moving and not nosedive
        rTd_unit << (rTd(0)/norm  + 0.25), rTd(1)/norm, rTd(2)/norm;

        //TODO: Tune it accordingly //so it rTd is not zero and it wont nose dive
        //double step_size = norm/5.0 + 0.25;
        double step_size = 1;
        ROS_INFO("norm: [%f], step size: [%f]", norm, step_size);
        Eigen::Vector3d target_pose_3d;
        target_pose_3d = rTd_unit * step_size;
        double elevation_global_frame = 0;

        grid_map::Matrix current_matrix = current_grid_map.get("elevation");
        fillIn(current_matrix);
        //std::cout<<"Current Elevation Matrix: \n"<<current_matrix<<std::endl;

        static double default_z = robot_current_pose.pose.position.z;
        double map_centre_x, map_centre_y;
        map_centre_x = current_grid_map.getSize()(0) / 2;
        map_centre_y = current_grid_map.getSize()(1) / 2;


        /*int grid_index_x = map_centre_x + target_pose_3d(0) / grid_resolution;
        int grid_index_y = map_centre_y + target_pose_3d(1) / grid_resolution;

        if((grid_index_x <current_grid_map.getSize()[0] || grid_index_y < current_grid_map.getSize()[1]) && (!std::isnan(current_matrix(grid_index_x, grid_index_y)))) {
            elevation_global_frame = current_matrix(grid_index_x, grid_index_y) + distance_to_maintain;
            default_z = elevation_global_frame;
        }
        else{
            elevation_global_frame = default_z;
            //ROS_INFO("Adding default value to elevation map");

        }*/

        Eigen::Matrix4d robot_target_pose_global_frame_4d, robot_target_pose_robot_frame_4d;
        robot_target_pose_robot_frame_4d << 1, 0, 0, target_pose_3d(0),
                0, 1, 0, target_pose_3d(1),
                0, 0, 1, 0,
                0, 0, 0, 1;
        robot_target_pose_global_frame_4d = robot_pose4d*robot_target_pose_robot_frame_4d;
        //double robot_pose_global_x =
        robot_target_pose.pose.position.x = robot_target_pose_global_frame_4d(0, 3);
        robot_target_pose.pose.position.y = robot_target_pose_global_frame_4d(1, 3);

        //EDIT ALG : TODO: I should use the point given by global x and y to get the height
        int grid_index_x = map_centre_x - 1; // + robot_target_pose_global_frame_4d(0, 3) / grid_resolution;

        int grid_index_y = map_centre_y - 1; // + robot_target_pose_global_frame_4d(1,3) / grid_resolution;

        //if((grid_index_x <current_grid_map.getSize()[0] || grid_index_y < current_grid_map.getSize()[1]) && (!std::isnan(current_matrix(grid_index_x, grid_index_y)))) {
        if(grid_index_x < current_grid_map.getSize()[0] && grid_index_y  < current_grid_map.getSize()[1]){
            elevation_global_frame = current_matrix(grid_index_x, grid_index_y) + distance_to_maintain;
            default_z = elevation_global_frame;
        }
        else{
            elevation_global_frame = default_z;
            //ROS_INFO("Adding default value to elevation map");

        }
        robot_target_pose.pose.position.z = elevation_global_frame;


        robot_target_pose.pose.orientation.w = 1;
        robot_target_pose.pose.orientation.x = 0;
        robot_target_pose.pose.orientation.y = 0;
        robot_target_pose.pose.orientation.z = 0;



        robot_target_pose.header.stamp = ros::Time::now();


    }


}
void generateTrajectoryUnit(){
    //get unit vectors along the general direction of diver



}


Eigen::Matrix<double, 3, 1> makeUnitVector(double x, double y, double z) {
    double norm = std::sqrt(x*x + y*y + z*z);
    return Eigen::Vector3d(x/norm, y/norm, z/norm);

}

Eigen::Vector3d getCombinedUnitVector(Eigen::Matrix<double, 3, 1> normal_vector) {
    Eigen::Vector3d z_vector(0, 0, 1);  //vector 1
    double combined_x, combined_y, combined_z;
    combined_x = z_vector(0)*zhiang_factor + normal_vector(0)*(1 - zhiang_factor);
    combined_y = z_vector(1)*zhiang_factor + normal_vector(1)*(1 - zhiang_factor);
    combined_z = z_vector(2)*zhiang_factor + normal_vector(2)*(1 - zhiang_factor);

    return makeUnitVector(combined_x, combined_y, combined_z);

}

int main(int argc, char **argv){
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;


    ros::Publisher pub_pose_command = nh.advertise<geometry_msgs::PoseStamped>("hippocampus/desired_pose", 1000);

    ros::Subscriber sub_elevation_map = nh.subscribe("normal_vector_generation/filtered_map", 1000, elevationMapCallback);

    //ros::Subscriber sub_elevation_map = nh.subscribe("/elevation_mapping/elevation_map", 1000, elevationMapCallback);

    ros::Subscriber sub_current_pose = nh.subscribe("/mavros/local_position/pose", 1000, robotPoseCallback);
    ros::Subscriber sub_diver_pose = nh.subscribe("hippocampus/diver_follower/diver_pose_filtered", 1000, diverPoseCallback);

    ros::Publisher rviz_path_pub = nh.advertise<nav_msgs::Path>("reference_path", 1000);

    ros::Rate rate(20);

    ros::Time previous_time = ros::Time::now();

    initializePoseCommand();

    while(current_grid_map.getSize()[0] <=0 ){
        //std::cout<<"here"<<std::endl;
        ros::spinOnce();
    }

    while(ros::ok()){

        nav_msgs::Path rviz_path_msg;
        generateTrajectoryUnitMethod();
        //generateLocalTrajectory(rviz_path_msg);
        //generateTrajectoryTRN(rviz_path_msg);
        //std::cout<<"Target Pose: "<<robot_target_pose<<std::endl;
        pub_pose_command.publish(robot_target_pose);

        rate.sleep();
        ros::spinOnce();
    }

    return 0;


}
