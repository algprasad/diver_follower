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
    Eigen::VectorXd x_;
    //covariance
    Eigen::MatrixXd cov_;
    //time step
    double del_t;
    //process noise
    Eigen::MatrixXd Q_;
    //Measurement Noise_;
    Eigen::MatrixXd R_;
    //Measurement funtion
    Eigen::MatrixXd H_;

    //epsilon to multiply Q_
    double epsilon;


    bool initialized_state;

    TrackingFilter()= default;
    TrackingFilter(double ros_freq){
        del_t = 1/ros_freq;
        initialized_state = false;
        epsilon = 0.0000000000005;

        //initial covariance
        cov_.resize(6,6);
        cov_ << 0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0;

        //Measurement Noise
        R_.resize(3,3);
        R_ << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
        //Process noise //TODO: source of error: constant process noise. Check if this doesnt work
        Q_.resize(6,6);
        const double t4 = pow(del_t, 4);
        const double t3 = pow(del_t, 3);
        const double t2 = pow(del_t, 2);
        /*Q_ <<  t4/4, 0, 0, t3/2, 0, 0,
                0, t4/4, 0, 0, t3/2, 0,
                0, 0, t4/4, 0, 0, t3/2,
                0, 0, 0, t2, 0, 0,
                0, 0, 0, 0, t2, 0,
                0, 0, 0, 0, 0, t2;*/
        Q_ << epsilon, 0, 0, 0, 0, 0,
              0, epsilon, 0, 0, 0, 0,
              0, 0, epsilon, 0, 0, 0,
              0, 0, 0, epsilon, 0, 0,
              0, 0, 0, 0, epsilon, 0,
              0, 0, 0, 0, 0, epsilon;

        H_.resize(3,6);
        H_ << 1, 0, 0, 0, 0, 0,
              0, 1, 0, 0, 0, 0,
              0, 0, 1, 0, 0, 0;

    }

    void initializeState(geometry_msgs::PoseStamped pose);

    void cycle(bool new_measurement, geometry_msgs::PoseStamped& measured_pose, geometry_msgs::PoseStamped& filtered_pose);

    void predict();

    void update(geometry_msgs::PoseStamped& measured_pose);

};


#endif //DIVER_FOLLOWER_TRACKINGFILTER_H
