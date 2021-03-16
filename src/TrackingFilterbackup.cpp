//
// Created by alg on 22/02/21.
//

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "TrackingFilterbackup.h"

void TrackingFilter::predict() {
    Eigen::MatrixXd A;
    A.resize(6,6);
    A << 1, 0, 0, del_t, 0, 0,
        0, 1, 0, 0, del_t, 0,
        0, 0, 1, 0, 0, del_t,
        0, 0, 0, 1, 0, 0,
        0, 0, 0, 0, 1, 0,
        0, 0, 0, 0, 0, 1;

    x_ = A*x_;
    cov_ = A*cov_*A.transpose() + Q_;

}

void TrackingFilter::update(geometry_msgs::PoseStamped& measured_pose) {

    Eigen::VectorXd Z, z_bar;
    Eigen::MatrixXd Z_cov;
    Z.resize(3);
    z_bar.resize(3);
    Z_cov.resize(3, 3);
    Z <<    measured_pose.pose.position.x,
            measured_pose.pose.position.y,
            measured_pose.pose.position.z;

    z_bar = Z - H_*x_;
    Z_cov = H_*cov_*H_.transpose() + R_;
    Eigen::MatrixXd K;
    K.resize(6,3);
    K = cov_*H_.transpose()*Z_cov.inverse();
    x_ = x_ + K*z_bar;
    cov_ = cov_ - K*H_*cov_;

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
    x_.resize(6);
    x_(0) = pose.pose.position.x;
    x_(1) = pose.pose.position.y;
    x_(2) = pose.pose.position.z;

    //TODO(source of error): Assigning velocities to zero
    x_(3) = 0;
    x_(4) = 0;
    x_(5) = 0;

    initialized_state = true;
}


