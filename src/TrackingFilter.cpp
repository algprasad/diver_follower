//
// Created by alg on 22/02/21.
//

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "TrackingFilter.h"

void TrackingFilter::predict() {
    Eigen::Matrix3d A;
    //A.resize(3,6);
    A << 1, 0, 0,
        0, 1, 0,
        0, 0, 1;

    x_ = A*x_;
    cov_ = A*cov_*A.transpose() + Q_;

}

void TrackingFilter::update(geometry_msgs::PoseStamped& measured_pose) {
    //TODO (How should I add velocity? ) // Add constant velocity for now and then use measured velocity
    //TODO: Remove the constant velocity values!!!!!!!!!!!!!!!
    Eigen::Vector3d Z, z_bar;
    Eigen::Matrix3d Z_cov;
    //Z.resize(6);
    //z_bar.resize(6);
    //Z_cov.resize(6,6);
    Z <<    measured_pose.pose.position.x,
            measured_pose.pose.position.y,
            measured_pose.pose.position.z;

    z_bar = Z - x_;
    Z_cov = cov_ + R_;
    Eigen::Matrix3d K;
    //K.resize(6,6);
    K = cov_*Z_cov.inverse();
    x_ = x_ + K*z_bar;
    cov_ = cov_ - K*Z_cov*K.transpose();

}


void TrackingFilter::cycle(bool new_measurement, geometry_msgs::PoseStamped& measured_pose, geometry_msgs::PoseStamped& filtered_pose) {
    predict();
    if(new_measurement) update(measured_pose);

    //update the
    filtered_pose.pose.position.x = this->x_(0);
    filtered_pose.pose.position.y = this->x_(1);
    filtered_pose.pose.position.z = this->x_(2);

    filtered_pose.header.stamp = ros::Time::now();
}

void TrackingFilter::initializeState(geometry_msgs::PoseStamped pose) {
    //x_.resize(6);
    x_(0) = pose.pose.position.x;
    x_(1) = pose.pose.position.y;
    x_(2) = pose.pose.position.z;

    initialized_state = true;
}


