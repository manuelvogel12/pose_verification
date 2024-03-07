#include "pose_verification/pose_verification.h"

#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


PoseVerification::PoseVerification(ros::NodeHandle nh):
  _nh(nh),
  _human_joint_sub(_nh.subscribe("/sara_shield/human_pose_measurement", 100, &PoseVerification::humanJointCallback, this)),
  _velodyne_sub(_nh.subscribe("/VLP16_lidar_front/velodyne_points", 100, &PoseVerification::velodyneCallback, this)),
  _model_state_sub(_nh.subscribe("/gazebo/model_states", 100, &PoseVerification::modelStatesCallback, this)),
  _line_marker_pub(_nh.advertise<visualization_msgs::Marker>("/pose_verification/line_list", 10)),
  _tfListener(_tfBuffer)
{
  last_message_time = ros::Time::now();
}


void PoseVerification::velodyneCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
  // get the lidar point cloud
  std::cout<<"Point Cloud Received"<<std::endl;
  std::cout<<"Size of Point cloud:"<<msg->width <<"x"<< msg->height<<std::endl;

  // debug visualization message 
  visualization_msgs::Marker line_list;
  line_list.header.frame_id = "VLP16_lidar_front";
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "pose_verification_line_list";
  line_list.id = 0;
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.1; // Line width
  line_list.color.r = 1.0; // Red
  line_list.color.a = 1.0; // Alpha (transparency)

  // convert to pcl for more efficient point cloud handling
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  
  // Convert pcl_pc2 to pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
  
  // Create a KD-Tree
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  kdtree.setInputCloud(cloud);

  // validate pose for every detected human
  for(int human_index = 0; human_index < MAX_NUM_HUMANS; human_index++){

    // use only detected humans
    if (!human_message_received[human_index]){
      continue;
    }

    // use chest and hands of the human as searchpoints
    for (int point_index : {6, 22, 23}) {
      geometry_msgs::Point humanPoint = _human_meas[human_index][point_index];

      pcl::PointXYZ nearestPointPCL = cloud->points[getClosestPCLPointIndex(kdtree, humanPoint)];
      geometry_msgs::Point nearestPoint;
      nearestPoint.x = nearestPointPCL.x; nearestPoint.y = nearestPointPCL.y; nearestPoint.z = nearestPointPCL.z;
      
      // make line between expected and actual location 
      line_list.points.push_back(humanPoint);
      line_list.points.push_back(nearestPoint);
    }
  }
  _line_marker_pub.publish(line_list);
}


int PoseVerification::getClosestPCLPointIndex(pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, geometry_msgs::Point& point){
  geometry_msgs::Point nearestPoint;
  int K = 1; // Number of closest points to search for
  std::vector<int> pointIdxNKNSearch(K);
  std::vector<float> pointNKNSquaredDistance(K);

  pcl::PointXYZ searchPoint;
  searchPoint.x = point.x; searchPoint.y = point.y; searchPoint.z = point.z;

  // nearest neighbor search
  if (kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    return pointIdxNKNSearch[0];
  }
  else{
    return 0;
  }
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
    ROS_INFO("NO TRANSFORM FOUND (ExtrapolationException)");
    return;
  }

  //get all human measurment points and transform them to the LiDAR coordinate system
  // TODO: do the transformation in the Lidar callback
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
      //std::cout<<"Human received"<<std::endl;
      human_message_received[human_index] = true;
    }
  }
}



//convert the gazebo transformation between world and base_link into a tf
void PoseVerification::modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  // avoid calling the function 2000 times per second
  if ((ros::Time::now() - last_message_time).toSec() < 0.01){
    return;
  }
  // std::cout<<"seconds passed"<<ros::Time::now() - last_message_time<<" seconds"<<std::endl;
  last_message_time = ros::Time::now();
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