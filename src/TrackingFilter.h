//
// Created by alg on 22/02/21.
//

#ifndef DIVER_FOLLOWER_TRACKINGFILTER_H
#define DIVER_FOLLOWER_TRACKINGFILTER_H


#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class TrackingFilter {
public:
    //state
    Eigen::Vector3d x_;
    //covariance
    Eigen::Matrix3d cov_;
    //time step
    double del_t;
    //process noise
    Eigen::Matrix3d Q_;
    //Measurement Noise_;
    Eigen::Matrix3d R_;

    bool initialized_state;

    TrackingFilter()= default;
    TrackingFilter(double ros_freq){
        del_t = 1/ros_freq;
        initialized_state = false;

        //initial covariance
        //cov_.resize(6,6);
        cov_ << 0, 0, 0,
                0, 0, 0,
                0, 0, 0;

        //Measurement Noise
        //R_.resize(6,6);
        R_ << 0.25, 0, 0,
                0, 0.25, 0,
                0, 0, 0.25;

        //Process noise //TODO: source of error: constant process noise. Check if this doesnt work
        //Q_.resize(6,6);
        Q_ <<  0.000005, 0, 0,
                0, 0.000005, 0,
                0, 0, 0.000005;
    }

    void initializeState(geometry_msgs::PoseStamped pose);

    void cycle(bool new_measurement, geometry_msgs::PoseStamped& measured_pose, geometry_msgs::PoseStamped& filtered_pose);

    void predict();

    void update(geometry_msgs::PoseStamped& measured_pose);

};


#endif //DIVER_FOLLOWER_TRACKINGFILTER_H
