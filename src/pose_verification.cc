#include "pose_verification/pose_verification.h"


PoseVerification::PoseVerification():
  nh{},
  _human_joint_sub(nh.subscribe("/sara_shield/human_pose_measurement", 100, &PoseVerification::humanJointCallback, this)),
  _velodyne_sub(nh.subscribe("/VLP16_lidar_front/velodyne_points", 100, &PoseVerification::velodyneCallback, this)),
  _tfListener(_tfBuffer)
{
}



void PoseVerification::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // get the lidar point cloud
  std::cout<<"Point Cloud Received"<<std::endl;
  std::cout<<"Size of Point cloud:"<<msg->width * msg->height<<std::endl;
}


void PoseVerification::humanJointCallback(const concert_msgs::HumansConstPtr& msg) {
  // get the robot position
  std::cout<<"Human received"<<std::endl;
  geometry_msgs::TransformStamped transformation;
  std::string source_frame =  msg->header.frame_id;
  if(source_frame == ""){
    source_frame="base_link";
  }

  try {
    transformation = _tfBuffer.lookupTransform(
        "base_link", source_frame, msg->header.stamp, ros::Duration(0.003));
  } catch (tf2::LookupException const&) {
    ROS_WARN("NO TRANSFORM FOUND (Lookup failed)");
    return;
  } catch (tf2::ExtrapolationException const&) {
    ROS_WARN("NO TRANSFORM FOUND (ExtrapolationException)");
    return;
  }

  //get all human measurment points and transform them to robot coordinate system
  if(msg->humans.size() > 0)
  {
    for (const concert_msgs::Human3D &human: msg->humans){
      _human_meas.clear();
      int human_index = human.label_id;
      for(const concert_msgs::Keypoint3D &keypoint:human.keypoints)
      {
        geometry_msgs::PointStamped pointStamped;
        geometry_msgs::PointStamped pointStampedLocal;
        pointStamped.point = keypoint.pose.position;    
        tf2::doTransform(pointStamped, pointStampedLocal, transformation);
        geometry_msgs::Point pointLocal = pointStampedLocal.point;
        
        _human_meas.emplace_back(pointLocal);

      }
      double messageTime = msg->header.stamp.toSec();
      if(messageTime == 0.0){
        messageTime = ros::Time::now().toSec();
      }
      // _shield.humanMeasurement(_human_meas, human_index, messageTime);

    }
  }
  
}


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "pose_verification_node");
    PoseVerification test();
    ros::spin();
    return 0;
}