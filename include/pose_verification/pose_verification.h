#ifndef POSEVERIFICATION2_H
#define POSEVERIFICATION2_H

#include <vector>

#include <ros/ros.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/PointCloud2.h>
#include <concert_msgs/Humans.h>
#include <gazebo_msgs/ModelStates.h>
#include <visualization_msgs/Marker.h>


class PoseVerification
{

public:

    PoseVerification(ros::NodeHandle nh);

    void velodyneCallback(const sensor_msgs::PointCloud2ConstPtr& msg);

    void humanJointCallback(const concert_msgs::HumansConstPtr& msg);

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg);

private:

    int getClosestPCLPointIndex(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, geometry_msgs::Point& point);

    ros::NodeHandle _nh;

    /**
     * @brief Fixed number of maximum allowed humans in the scene
     */
    const static int MAX_NUM_HUMANS = 5; 

    /**
     * @brief Human measurement points
     */
    std::vector<geometry_msgs::Point> _human_meas[MAX_NUM_HUMANS];

    /**
     * @brief The last receiving time of the human i 
     */
    double _human_meas_time[MAX_NUM_HUMANS];

    /**
     * @brief Whether the human with index i has been received yet 
     */
    bool human_message_received[MAX_NUM_HUMANS] = {false};

    /**
     * @brief Time of last received modelStatesCallback message 
     */
    ros::Time last_message_time;

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

    /**
     * @brief Ros publisher to publish debug markers showing the distance 
     * from the expected human point to the closest lidar point
     */
    ros::Publisher _line_marker_pub;
};



#endif