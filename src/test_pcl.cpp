//
// Created by alg on 15/09/20.
//
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

void callback(const PointCloud::ConstPtr& msg)
{
    std::cout<<"callback"<<std::endl; 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sub_pcl");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<PointCloud>("/stereo/points2", 1, callback);
    ros::Rate rate(20); 
    while(ros::ok()){
        std::cout<<"while"<<std::endl;
        ros::spinOnce();    
    }
    
}
