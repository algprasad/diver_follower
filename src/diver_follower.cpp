#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>
#include "eigen3/Eigen/Geometry"

using namespace message_filters;

//load the internal camera matrix
double fx_l = 238.35154, fy_l= 238.35154, fx_r= 238.35154, fy_r = 238.35154;
double shear_l = 0, shear_r= 0;
double px_l= 200.5, py_l= 200.5, px_r = 200.5, py_r= 200.5;

//stereo camera parameters
double baseline = 0.07; //7cm

struct Pixel {
    long u;
    long v;
    Pixel(): u(0), v(0){}
    Pixel(long x, long y): u(x), v(y){}
};

geometry_msgs::PoseStamped target_robot_pose;
geometry_msgs::PoseStamped current_robot_pose;

void callback(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes_left,
        const darknet_ros_msgs::BoundingBoxesConstPtr& boxes_right)//,
       // const geometry_msgs::PoseStampedConstPtr& robot_pose)
{
    darknet_ros_msgs::BoundingBox box_left =  boxes_left->bounding_boxes[0];
    darknet_ros_msgs::BoundingBox box_right =  boxes_right->bounding_boxes[0];
    Pixel left_centre, right_centre;
    left_centre.u = (box_left.xmax + box_left.xmin)/2;
    left_centre.v = (box_left.ymax + box_left.ymin)/2;

    right_centre.u = (box_right.xmax + box_right.xmin)/2;
    right_centre.v = (box_right.ymax + box_right.ymin)/2;

    //load the internal camera matrix
    double fx_l = 238.35154, fy_l= 238.35154, fx_r= 238.35154, fy_r = 238.35154;
    double shear_l = 0, shear_r= 0;
    double px_l= 200.5, py_l= 200.5, px_r = 200.5, py_r= 200.5;

    //stereo camera parameters
    double baseline = 0.07; //7cm

    //perfom calculations for the getting x, y, z coordinates
    double disparity = left_centre.u - right_centre.u;

    //Z
    double Z = (fx_l*baseline)/(disparity);

    //Y
    double Y = (left_centre.v - py_l)*Z/fy_l;

    //X
    double X = (left_centre.u - px_l - (shear_l/fy_l)*(left_centre.v - py_l))*(Z/fx_l);


    //std::cout<<"position of diver: "<<X<<" "<<Y<<" "<<Z<<std::endl;


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
    Eigen::Matrix4d camera_opt_wrt_baselink_4d;
    ///Fixed for the hiipocampus model. Maybe use the .yaml file
    camera_opt_wrt_baselink_4d << 0, 0, 1, 0.2,
                                  0, 1, 0, 0,
                                  -1, 0, 0, 0.03,
                                  0, 0, 0, 1;

    Eigen::Matrix4d diver_wrt_camera_opt_4d;
    diver_wrt_camera_opt_4d << 1, 0, 0, X,
                                0, 1, 0, Y,
                                0, 0, 1, Z,
                                0, 0, 0, 1;

    Eigen::Matrix4d diver_global_pose_4d;
    diver_global_pose_4d = robot_pose_4d*camera_opt_wrt_baselink_4d*diver_wrt_camera_opt_4d;


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
    d2m_vector <<   d2m*unit_distance_vector(0),
                    d2m*unit_distance_vector(1),
                    d2m*unit_distance_vector(2);

    Eigen::Vector3d position_setpoint;
    position_setpoint << diver_global_position(0) + d2m_vector(0),
                        diver_global_position(1) + d2m_vector(1),
                        diver_global_position(2);

    target_robot_pose.pose.position.x = position_setpoint(0);
    target_robot_pose.pose.position.y = position_setpoint(1);
    target_robot_pose.pose.position.z = position_setpoint(2);
    //std::cout<<"TARGET POSE: "<<target_robot_pose.pose.position.x<<" "<<
   // target_robot_pose.pose.position.y<<" "<<
    //target_robot_pose.pose.position.z<<std::endl;
}


void callback_updated(const darknet_ros_msgs::BoundingBoxesConstPtr& boxes_left,
              const darknet_ros_msgs::BoundingBoxesConstPtr& boxes_right)//,
// const geometry_msgs::PoseStampedConstPtr& robot_pose)
{
    std::cout<<"New callback is being called"<<std::endl;
    darknet_ros_msgs::BoundingBox box_left = boxes_left->bounding_boxes[0];
    darknet_ros_msgs::BoundingBox box_right = boxes_right->bounding_boxes[0];
    int num_loop_points = box_left.ymax - box_left.ymin; //height of bounding box is assumed (maybe correctly) same in both cases
    int k =100;
    long x_mid_left = (box_left.xmin + box_left.xmax)/2;
    long x_mid_right = (box_right.xmin + box_right.xmax)/2;


    double sum_z = 0, sum_x = 0, sum_y =0;
    int num_valid_depths = 0;
    while(k<num_loop_points - 100){
        long y_left = box_left.ymin + k;
        long y_right = box_right.ymin + k;
        Pixel current_left(x_mid_left, y_left), current_right(x_mid_right, y_right);
        double disparity = current_left.u - current_right.u;
        //Z
        double Z = (fx_l*baseline)/(disparity);

        //Y
        double Y = (current_left.v - py_l)*Z/fy_l;

        //X
        double X = (current_left.u - px_l - (shear_l/fy_l)*(current_left.v - py_l))*(Z/fx_l);

        sum_z+=Z;
        sum_x+=X;
        sum_y+=Y;
        num_valid_depths++;
        k++;
    }

    double average_depth = sum_z/num_valid_depths;
    double average_x = sum_x/num_valid_depths;
    double average_y = sum_y/num_valid_depths;

    std::cout<<"Position of diver wrt camera: "<<average_x<<" "<<average_y<<" "<<average_depth<<std::endl;

}
void initializeRobotPose(){
    target_robot_pose.pose.position.x = 0;
    target_robot_pose.pose.position.y = 0;
    target_robot_pose.pose.position.z = 0;



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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "diver_follower_node");

    ros::NodeHandle nh;

    //add the publisher to publish setpoint for the drone to follow
    ros::Publisher pub_robot_target_pose = nh.advertise<geometry_msgs::PoseStamped>("hippocampus/desired_pose", 10);
    ros::Subscriber sub_current_pose = nh.subscribe("/mavros/local_position/pose", 1000, mavrosGlobalPositionCallback);

    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> image_sub_left(nh, "/darknet_ros_left/bounding_boxes", 10);
    message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> image_sub_right(nh, "/darknet_ros/bounding_boxes", 10);
    message_filters::Subscriber<geometry_msgs::PoseStamped> robot_sub(nh, "/mavros/local_position/pose", 10);

    //TimeSynchronizer<darknet_ros_msgs::BoundingBoxes, darknet_ros_msgs::BoundingBoxes, geometry_msgs::PoseStamped > sync(image_sub_left, image_sub_right, robot_sub,  10);
    TimeSynchronizer<darknet_ros_msgs::BoundingBoxes, darknet_ros_msgs::BoundingBoxes > sync(image_sub_left, image_sub_right,  10);
    //sync.registerCallback(boost::bind(&callback, _1, _2, _3));
    sync.registerCallback(boost::bind(&callback_updated, _1, _2));

    initializeRobotPose();
    //ros::spin();
    ros::Rate rate(20);
    while(ros::ok()){
        pub_robot_target_pose.publish(target_robot_pose);
        ros::spinOnce();
        rate.sleep();
    }



    return 0;
}
