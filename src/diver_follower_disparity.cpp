//
// Created by alg on 14/09/20.
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



static const std::string OPENCV_WINDOW = "Image window";
using namespace message_filters;
double focal_length = 238.3515;
double baseline = 0.07;
double px_l= 200.5, py_l= 200.5;
double shear_l = 0;

geometry_msgs::PoseStamped target_robot_pose;
geometry_msgs::PoseStamped current_robot_pose;

std::string type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}


void disparityCallback(const stereo_msgs::DisparityImageConstPtr& msg){
    sensor_msgs::Image disparity_image = msg->image;


    //convert to opencv message
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(disparity_image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
        cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    cv::Mat mat_image  =cv_ptr->image;
    float pixel_value = mat_image.at<float>(300,300);
    //std::cout<<pixel_value<<std::endl;
    double focal_length = 238.3515;
    double baseline = 0.07;

    double deepth  = focal_length*baseline/pixel_value;
    //std::cout<<deepth<<std::endl;

}
void setDronePose(double diver_x_wrtc, double diver_y_wrtc, double diver_z_wrtc){

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

    //std::cout<<"Robot Pose 4d: "<<robot_pose_4d<<std::endl;
    Eigen::Matrix4d camera_opt_wrt_baselink_4d;
    ///Fixed for the hiipocampus model. Maybe use the .yaml file
    camera_opt_wrt_baselink_4d << 0, 0, 1, 0.2,
            0, 1, 0, 0,
            -1, 0, 0, 0.03,
            0, 0, 0, 1;

    Eigen::Matrix4d diver_wrt_camera_opt_4d;
    diver_wrt_camera_opt_4d << 1, 0, 0, diver_x_wrtc,
            0, 1, 0, diver_y_wrtc,
            0, 0, 1, diver_z_wrtc,
            0, 0, 0, 1;

    Eigen::Matrix4d diver_global_pose_4d;
    diver_global_pose_4d = robot_pose_4d*camera_opt_wrt_baselink_4d*diver_wrt_camera_opt_4d;
    //std::cout<<"Diver Global pose: "<<diver_global_pose_4d<<std::endl;

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
    d2m_vector <<   -d2m*unit_distance_vector(0),
            -d2m*unit_distance_vector(1),
            -d2m*unit_distance_vector(2);

    Eigen::Vector3d position_setpoint;
    position_setpoint << diver_global_position(0) + d2m_vector(0),
            diver_global_position(1) + d2m_vector(1),
            diver_global_position(2) -1 ;

    target_robot_pose.pose.position.x = position_setpoint(0);
    target_robot_pose.pose.position.y = position_setpoint(1);
    target_robot_pose.pose.position.z = position_setpoint(2);
    /*std::cout<<"TARGET POSE: "<<target_robot_pose.pose.position.x<<" "<<
             target_robot_pose.pose.position.y<<" "<<
             target_robot_pose.pose.position.z<<std::endl;*/


}
void boundingBoxDisparityImageCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bounding_boxes_msg,
        const stereo_msgs::DisparityImageConstPtr& disparity_image_msg){

    //bounding boxes part
    darknet_ros_msgs::BoundingBox box_left =  bounding_boxes_msg->bounding_boxes[0];

    //get the list of all points
    int x_mid = (box_left.xmax + box_left.xmin)/2;
    int y_mid = (box_left.ymax + box_left.ymin)/2;

    double img_height = disparity_image_msg->image.height;
    double img_width = disparity_image_msg->image.width;


    //disparity image part
    sensor_msgs::Image disparity_image = disparity_image_msg->image;


    //convert to opencv message
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(disparity_image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat mat_image  =cv_ptr->image;


    //combination part
    double sum_depth = 0, sum_x = 0, sum_y =0;
    int num_valid_depths = 0;
    /*for(int j = box_left.ymin; j <=box_left.ymax; j++){
        float current_pixel_value = mat_image.at<float>(x_mid, j);
        double depth = 0;
        if(current_pixel_value != -1 && current_pixel_value!= 0){
            depth  = focal_length*baseline/current_pixel_value;
            double Z = depth;

            //Y
            double Y = (j - py_l)*Z/focal_length;

            //X
            double X = (x_mid - px_l - (shear_l/focal_length)*(j - py_l))*(Z/focal_length);
            sum_depth+=depth;
            sum_x+=X;
            sum_y+=Y;
            num_valid_depths++;
        }

    }*/
    for(int i=box_left.xmin; i<box_left.xmax; i++){
        float current_pixel_value = mat_image.at<float>(i, y_mid);
        double depth = 0;
        if(current_pixel_value != -1 && current_pixel_value!= 0){
            depth  = focal_length*baseline/current_pixel_value;
            double Z = depth;

            //Y
            double Y = (y_mid - py_l)*Z/focal_length;

            //X
            double X = (i - px_l - (shear_l/focal_length)*(y_mid - py_l))*(Z/focal_length);
            sum_depth+=depth;
            sum_x+=X;
            sum_y+=Y;
            num_valid_depths++;
        }

    }

    if (num_valid_depths >0) {
        double average_depth = sum_depth/num_valid_depths;
        double average_x = sum_x/num_valid_depths;
        double average_y = sum_y/num_valid_depths;
        std::cout<<"Position of diver wrt camera: "<<average_x<<" "<<average_y<<" "<<average_depth<<std::endl;
        if(!std::isnan(average_x) && !std::isnan(average_y) && !std::isnan(average_depth)) setDronePose(average_x, average_y, average_depth);

    }


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

void initializeRobotPose(){
    target_robot_pose.pose.position.x = 0;
    target_robot_pose.pose.position.y = 0;
    target_robot_pose.pose.position.z = 0;
}


void depthMapCallback(const sensor_msgs::ImageConstPtr& depth_image_msg){
    //disparity image part
    sensor_msgs::Image depth_image = *depth_image_msg;


    //convert to opencv message
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat mat_image  =cv_ptr->image;
    // Update GUI Window
    cv::imshow("Depth Image", cv_ptr->image);
    cv::waitKey(3);






}

void depthMap2Callback(const sensor_msgs::ImageConstPtr& msg){

    //disparity image part
    sensor_msgs::Image depth_image = *msg;


    //convert to opencv message
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depth_image);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat mat_image  =cv_ptr->image;
    // Update GUI Window
    cv::imshow("Depth Image", cv_ptr->image);
    cv::waitKey(3);
    std::cout<<"Depth Image type: "<<type2str(cv_ptr->image.type())<<std::endl;



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

void depthMap3Callback(const sensor_msgs::PointCloud2ConstPtr& msg){
    std::cout<<"callback3. Proof: "<<std::endl;






}

int main(int argc, char** argv){
    ros::init(argc, argv, "diver_follower_with_disparity");
    ros::NodeHandle nh;

    //keep the subscriber. write the synced callback separately
    //ros::Subscriber sub_depth_temp = nh.subscribe("/stereo/disparity", 100, disparityCallback);
    ros::Subscriber sub_current_pose = nh.subscribe("/mavros/local_position/pose", 1000, mavrosGlobalPositionCallback);
    //ros::Subscriber sub_depth_map = nh.subscribe("/camera/depth/cloud_image", 10, depthMapCallback);
    ros::Subscriber sub_depth_map2 = nh.subscribe("/camera/depth/cloud_image", 10, depthMap2Callback);
    //ros::Subscriber sub_depth_map3 = nh.subscribe("/stereo/points2", 10, depthMap3Callback);

    ros::Publisher pub_robot_target_pose = nh.advertise<geometry_msgs::PoseStamped>("hippocampus/desired_pose", 10);


    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bounding_box(nh, "/darknet_ros/bounding_boxes", 10);
    message_filters::Subscriber<stereo_msgs::DisparityImage> sub_disparity_image(nh,"/stereo/disparity", 10 );

    typedef sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, stereo_msgs::DisparityImage > MySyncPolicy;
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_bounding_box, sub_disparity_image);
    sync.registerCallback(boost::bind(&boundingBoxDisparityImageCallback, _1, _2));

    initializeRobotPose();

    ros::Rate rate(20);
    while(ros::ok()){
        pub_robot_target_pose.publish(target_robot_pose);
        ros::spinOnce();
        rate.sleep();
    }

}