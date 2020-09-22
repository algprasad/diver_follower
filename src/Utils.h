//
// Created by alg on 21/09/20.
//

#ifndef DIVER_FOLLOWER_UTILS_H
#define DIVER_FOLLOWER_UTILS_H

#include <tf/transform_broadcaster.h>

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

int shaveOffMin(int perc, int min, int max){
    return (min + ((perc/100)*(max- min)));
}

int shaveOffMax(int perc, int min, int max){
    return (max - ((perc/100)*(max - min)));
}

void publishTf(geometry_msgs::PoseStamped pose, std::string name){
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z) );
  tf::Quaternion q;
  q.setX(pose.pose.orientation.x);
  q.setY(pose.pose.orientation.y);
  q.setZ(pose.pose.orientation.z);
  q.setW(pose.pose.orientation.w);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));

}

void publishCameraTf(geometry_msgs::PoseStamped drone_pose){

}

#endif //DIVER_FOLLOWER_UTILS_H
