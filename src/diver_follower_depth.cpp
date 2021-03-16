//
// Created by alg on 17/09/20.
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
#include <math.h>


geometry_msgs::PoseStamped target_robot_pose;
geometry_msgs::PoseStamped current_robot_pose;

Eigen::Vector3d diver_global_position, robot_global_position, diver_global_position_measurement_new, diver_global_position_measurement_old;
Eigen::Vector3d vk;
Eigen::Matrix3d Pk;
ros::Time previous_time;

//to keep track of filtered_k
static int k = 0;

//boolean to know if a new measurement of diver pose is available
bool new_diver_pose = false;

//to keep track of delta time


using namespace message_filters;


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

void initializeRobotPose(){
    target_robot_pose.pose.position.x = 0;
    target_robot_pose.pose.position.y = 0;
    target_robot_pose.pose.position.z = 0;

    //initialize diver global position
    diver_global_position << 5, 0, 0;  //TODO check this one

    //initialize old diver global position measurement
    diver_global_position_measurement_old = diver_global_position;

    //initialize vk
    vk << 0, 0, 0;

    // initialize Pk
    Pk <<   0.01, 0, 0,
            0, 0.01, 0,
            0, 0, 0.01;

}

int shaveOffMin(int perc, int min, int max){
    return (min + ((perc/100)*(max- min)));
}

int shaveOffMax(int perc, int min, int max){
    return (max - ((perc/100)*(max - min)));
}

void pixelTo3DPoint(const sensor_msgs::PointCloud2 pCloud, const int u, const int v, geometry_msgs::Point &p)
{
// get width and height of 2D point cloud data
    int width = pCloud.width;
    int height = pCloud.height;

// Convert from u (column / width), v (row/height) to position in array
// where X,Y,Z data starts
    int arrayPosition = v*pCloud.row_step + u*pCloud.point_step;

// compute position in array where x,y,z data start
    int arrayPosX = arrayPosition + pCloud.fields[0].offset; // X has an offset of 0
    int arrayPosY = arrayPosition + pCloud.fields[1].offset; // Y has an offset of 4
    int arrayPosZ = arrayPosition + pCloud.fields[2].offset; // Z has an offset of 8

    float X = 0.0;
    float Y = 0.0;
    float Z = 0.0;

    memcpy(&X, &pCloud.data[arrayPosX], sizeof(float));
    memcpy(&Y, &pCloud.data[arrayPosY], sizeof(float));
    memcpy(&Z, &pCloud.data[arrayPosZ], sizeof(float));

    p.x = X;
    p.y = Y;
    p.z = Z;

}

void setDroneTargetPose(){

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
            //diver_global_position(2);
            current_robot_pose.pose.position.z;
    target_robot_pose.pose.position.x = position_setpoint(0);
    target_robot_pose.pose.position.y = position_setpoint(1);
    target_robot_pose.pose.position.z = position_setpoint(2);
    std::cout<<"TARGET POSE: "<<target_robot_pose.pose.position.x<<" "<<
             target_robot_pose.pose.position.y<<" "<<
             target_robot_pose.pose.position.z<<std::endl;

}

Eigen::Vector3d getVelocityMeasurement(){
    //get the time elapsed since the last position measurement
    if(k == 0){
        k++;
        previous_time = ros::Time::now();
        return Eigen::Vector3d(0, 0, 0); //since it's the first meaurement, use it to calibrate previous_time
    }

    ros::Duration time_elapsed = ros::Time::now() - previous_time;
    double delta_t = time_elapsed.toSec();
    Eigen::Vector3d velocity_measurement = (diver_global_position_measurement_new - diver_global_position_measurement_old)/delta_t;

    //update the previous values to the current values
    diver_global_position_measurement_old = diver_global_position_measurement_new;
    previous_time = ros::Time::now();
    return velocity_measurement;


}
void filterDiverPose(){
    Eigen::Vector3d vk_plus_1;
    Eigen::Matrix3d kalman_gain;

    //params
    double delta_t = 1.0/30.0;
    Eigen::Matrix3d Q, R, H;
    Q << 0.02, 0, 0,
         0, 0.02, 0,
         0, 0, 0.02;

    R << 0.05, 0, 0,
         0, 0.05, 0,
         0, 0, 0.05;

    H << 1, 0, 0,  //measurement equation matrix. since we are directly measuring velocity hence H is identity
         0, 1, 0,
         0, 0, 1;


    //process step
    Eigen::Vector3d vk_minus;
    Eigen::Matrix3d Pk_minus;
    //step 1
    vk_minus = vk;
    //step 2
    Pk_minus = Pk + Q;

    //measurement steps
    if(new_diver_pose){
        //step 3
        Eigen::Matrix3d Pkminus_plus_R = Pk_minus + R;
        Eigen::Matrix3d Pkminus_plus_R_inv = Pkminus_plus_R.inverse();
        kalman_gain = Pk_minus * Pkminus_plus_R_inv;

        //estimate measurement
        Eigen::Vector3d zk = getVelocityMeasurement();

        //step 4
        Eigen::Vector3d Hkvkminus =  zk - H*vk_minus;
        vk = vk_minus + kalman_gain*Hkvkminus;
        Eigen::Matrix3d identity_mat;
        identity_mat << 1, 0, 0,
                        0, 1, 0,
                        0, 0, 1;

        //step 5
        Pk = (identity_mat - kalman_gain*H)*Pk_minus;
        new_diver_pose = false;
    }

    else{ // in case the process step keeps going on
        Pk = Pk_minus;
        vk = vk_minus;
    }

    //set diver's global position using the estimate from vk
    diver_global_position = diver_global_position + vk*delta_t;

    //set target pose of the drone
    setDroneTargetPose();
    k++;

}



void calculateDiverGlobalPose(double diver_x_wrtc, double diver_y_wrtc, double diver_z_wrtc){

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
                                  0, 1, 0, 0,
                                  -1, 0, 0, 0.03,
                                  0, 0, 0, 1;

    Eigen::Matrix4d diver_wrt_camera_opt_4d;
    diver_wrt_camera_opt_4d <<  1, 0, 0, diver_x_wrtc,
                                0, 1, 0, diver_y_wrtc,
                                0, 0, 1, diver_z_wrtc,
                                0, 0, 0, 1;

    Eigen::Matrix4d diver_global_pose_4d;
    diver_global_pose_4d = robot_pose_4d*camera_opt_wrt_baselink_4d*diver_wrt_camera_opt_4d;
    //std::cout<<"Diver Global pose: "<<diver_global_pose_4d<<std::endl;



    diver_global_position_measurement_new << diver_global_pose_4d(0, 3), diver_global_pose_4d(1, 3), diver_global_pose_4d(2, 3);
    robot_global_position << robot_pose_4d(0,3), robot_pose_4d(1,3), robot_pose_4d(2,3);

    new_diver_pose = true;

}


void boundingBoxDepthImageCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_boxes_msg,
                                   const sensor_msgs::PointCloud2ConstPtr& point_cloud_msg){

    darknet_ros_msgs::BoundingBox bbox_main =  bounding_boxes_msg->bounding_boxes[0];
    const sensor_msgs::PointCloud2 main_point_cloud = *point_cloud_msg;

    //get the smaller bounding box with x% and y% shaved off
    int x_min, y_min, x_max, y_max;
    x_min = shaveOffMin(20, bbox_main.xmin, bbox_main.xmax);
    x_max = shaveOffMax(20, bbox_main.xmin, bbox_main.xmax);
    y_min = shaveOffMin(30, bbox_main.ymin, bbox_main.ymax);
    y_max = shaveOffMax(30, bbox_main.ymin, bbox_main.ymax);

    std::vector<geometry_msgs::Point> points3d_inside_box;

    //get 3d values of all points inside the shaved off bounding box
    int count_total_points = 0;
    int count_valid_points = 0;
    double sum_x = 0, sum_y = 0, sum_z = 0;
    for(int u = x_min; u < x_max; u++){
        for(int v = y_min; v < y_max; v++){
            geometry_msgs::Point temp_point;
            pixelTo3DPoint(main_point_cloud, u, v, temp_point);
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
    std::cout<<"Average Point: "<<average_x<<" "<<average_y<<" "<<average_z<<std::endl;

    calculateDiverGlobalPose(average_x, average_y, average_z);

}


int main(int argc, char** argv){
    ros::init(argc, argv, "diver_follower_with_depth");
    ros::NodeHandle nh;

    ros::Subscriber sub_current_pose = nh.subscribe("/mavros/local_position/pose", 1000, mavrosGlobalPositionCallback);
    ros::Publisher pub_robot_target_pose = nh.advertise<geometry_msgs::PoseStamped>("hippocampus/desired_pose", 10);


    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bounding_box(nh, "/darknet_ros/bounding_boxes", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_depth_points(nh, "/stereo/points2", 10 );

    typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2 > MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync2(MySyncPolicy(10), sub_bounding_box, sub_depth_points);
    sync2.registerCallback(boost::bind(&boundingBoxDepthImageCallback, _1, _2));

    initializeRobotPose();

    ros::Rate rate(30);
    while(ros::ok()){
        //produce continuous estimates of diver pose. When the measurement is available update the diver pose
        filterDiverPose();
        std::cout<<"Diver global pose: "<<diver_global_position<<std::endl; 
        pub_robot_target_pose.publish(target_robot_pose);
        ros::spinOnce();
        rate.sleep();
    }

}