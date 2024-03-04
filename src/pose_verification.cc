#include "pose_verification/pose_verification.h"

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>


PoseVerification::PoseVerification(ros::NodeHandle nh):
  _nh(nh),
  _human_joint_sub(_nh.subscribe("/sara_shield/human_pose_measurement", 100, &PoseVerification::humanJointCallback, this)),
  _velodyne_sub(_nh.subscribe("/VLP16_lidar_front/velodyne_points", 100, &PoseVerification::velodyneCallback, this)),
  _model_state_sub(_nh.subscribe("/gazebo/model_states", 100, &PoseVerification::modelStatesCallback, this)),
  _velodyne_human_pub(_nh.advertise<sensor_msgs::PointCloud2>("/TEST/velodyne", 100)),
  _tfListener(_tfBuffer)
{
}


void PoseVerification::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // get the lidar point cloud
  std::cout<<"Point Cloud Received"<<std::endl;
  std::cout<<"Size of Point cloud:"<<msg->width <<"x"<< msg->height<<std::endl;
  
  // debug output point cloud 
  sensor_msgs::PointCloud2 humanPointCloud;
  int count = 0;

  // convert to pcl for more efficient point cloud handling
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);

  for(int human_index = 0; human_index < 2; human_index++){
    for(geometry_msgs::Point& humanPoint:_human_meas[human_index]){
      // TODO find all points close to a human 
    }
  }
   

  // send out debug pointcloud
  humanPointCloud.header = msg->header;
  humanPointCloud.width = count;
  humanPointCloud.height = 1;
  _velodyne_human_pub.publish(humanPointCloud);
}


void PoseVerification::humanJointCallback(const concert_msgs::HumansConstPtr& msg) {
  // get the world to LiDAR transformation 
  geometry_msgs::TransformStamped transformation;
  std::string target_frame = "VLP16_lidar_front";
  std::string source_frame = msg->header.frame_id;
  if(source_frame == ""){
    source_frame="base_link";
  }

  try {
    transformation = _tfBuffer.lookupTransform(
        target_frame, source_frame, msg->header.stamp, ros::Duration(0.02));
  } catch (tf2::LookupException const&) {
    ROS_WARN("NO TRANSFORM FOUND (Lookup failed)");
    return;
  } catch (tf2::ExtrapolationException const&) {
    ROS_WARN("NO TRANSFORM FOUND (ExtrapolationException)");
    return;
  }

  //get all human measurment points and transform them to the LiDAR coordinate system
  if(msg->humans.size() > 0){
    for (const concert_msgs::Human3D &human: msg->humans){
      int human_index = human.label_id;
      _human_meas[human_index].clear();
      for(const concert_msgs::Keypoint3D &keypoint:human.keypoints){
        geometry_msgs::PointStamped pointStamped;
        geometry_msgs::PointStamped pointStampedLocal;
        pointStamped.point = keypoint.pose.position;
        // transform human points to lidar coordinates
        tf2::doTransform(pointStamped, pointStampedLocal, transformation);
        geometry_msgs::Point pointLocal = pointStampedLocal.point;
        
        _human_meas[human_index].emplace_back(pointLocal);
        _human_meas_time[human_index] = msg->header.stamp.toSec();

      }
      std::cout<<"Human received"<<std::endl;
    }
  } 
}



//convert the gazebo transformation between world and base_link into a tf
void PoseVerification::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  ROS_DEBUG("Incoming Transform");
  // find the index of the robot transform in the list of transforms
  int index = -1;
  for (uint64_t i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "ModularBot") {
      index = i;
      break;
    }
  }

  if (index == -1) {
    ROS_ERROR("Could not find robot modular_robot in model states message");
    return;
  }
  static tf2_ros::TransformBroadcaster br;

  // make a tf transform out of the given gazebo transform
  geometry_msgs::TransformStamped TransformStamped;
  TransformStamped.header.stamp = ros::Time::now();
  TransformStamped.header.frame_id = "world";
  TransformStamped.child_frame_id = "base_link";
  TransformStamped.transform.translation.x = msg->pose[index].position.x;
  TransformStamped.transform.translation.y = msg->pose[index].position.y;
  TransformStamped.transform.translation.z = msg->pose[index].position.z;
  TransformStamped.transform.rotation = msg->pose[index].orientation;

  //publish the transform
  br.sendTransform(TransformStamped);
}



int main(int argc, char * argv[]){
  ros::init(argc, argv, "pose_verification_node");
  ros::NodeHandle nh;
  PoseVerification pose(nh);
  ROS_WARN("Start");
  ros::spin();
  return 0;
}