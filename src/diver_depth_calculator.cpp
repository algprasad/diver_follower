//
// Created by alg on 18/09/20.
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

using namespace message_filters;
geometry_msgs::PoseStamped diver_pose_wrt_camera;

bool callbacks_called = false;
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
    diver_pose_wrt_camera.pose.position.x = average_x;
    diver_pose_wrt_camera.pose.position.y = average_y;
    diver_pose_wrt_camera.pose.position.z = average_z;
    diver_pose_wrt_camera.header.stamp = ros::Time::now();

    callbacks_called = true;


}
int main(int argc, char** argv){
    ros::init(argc, argv, "diver_depth_calculator");
    ros::NodeHandle nh;

    ros::Publisher pub_diver_pose = nh.advertise<geometry_msgs::PoseStamped>("hippocampus/diver_pose_wrt_camera", 10);


    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> sub_bounding_box(nh, "/darknet_ros/bounding_boxes", 10);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub_depth_points(nh, "/stereo/points2", 10 );

    typedef message_filters::sync_policies::ApproximateTime<darknet_ros_msgs::BoundingBoxes, sensor_msgs::PointCloud2 > MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync2(MySyncPolicy(10), sub_bounding_box, sub_depth_points);
    sync2.registerCallback(boost::bind(&boundingBoxDepthImageCallback, _1, _2));


    ros::Rate rate(20);
    while(ros::ok()){

        //produce continuous estimates of diver pose. When the measurement is available update the diver pose

        if(callbacks_called){
            std::cout << "Diver pose wrt camera: " << diver_pose_wrt_camera << std::endl;
            pub_diver_pose.publish(diver_pose_wrt_camera);
        }

        ros::spinOnce();
        //rate.sleep();
    }

}