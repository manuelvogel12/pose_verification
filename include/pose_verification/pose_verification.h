#ifndef POSEVERIFICATION2_H
#define POSEVERIFICATION2_H

#include <vector>

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <concert_msgs/Humans.h>
#include <gazebo_msgs/ModelStates.h>


class PoseVerification
{

public:

    PoseVerification(ros::NodeHandle nh);

    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    void humanJointCallback(const concert_msgs::HumansConstPtr& msg);

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);


private:

    ros::NodeHandle _nh;

    /**
     * @brief Human measurement points to be used in sara shield
     */
    std::vector<geometry_msgs::Point> _human_meas[2];
    //TODO: fix for any amount of humans

    double _human_meas_time[2];


    /**
     * @brief tf2 transformation buffer to lookup the position and orientation of the robot
     */
    tf2_ros::Buffer _tfBuffer;
       
    /**
     * @brief tf2 Listener to get transforms
     */
    tf2_ros::TransformListener _tfListener;


    /**
     * @brief Ros subsriber to receveive human joint positions
     */
    ros::Subscriber _human_joint_sub;

    /**
     * @brief Ros subsriber to receveive velodyne point cloud
     */
    ros::Subscriber _velodyne_sub;


    /**
     * @brief Ros subscriber to receive Robot position and orientation
     */
    ros::Subscriber _model_state_sub;

    ros::Publisher _velodyne_human_pub;

};



#endif