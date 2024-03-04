#pragma once

#include "KeyFrame.h"
#include "Map.h"
#include "System.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "interfaces/msg/is_lost_from_base_map.hpp"
#include "interfaces/msg/loop_closure_triggers.hpp"
#include "interfaces/msg/new_key_frame_bows.hpp"
#include "interfaces/msg/new_key_frames.hpp"
#include "interfaces/msg/successfully_merged.hpp"
#include "interfaces/msg/uuid.hpp"
#include "interfaces/srv/get_current_map.hpp"
#include "interfaces/srv/get_map_points.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "peer.h"
#include "publish_ros_viz_topics.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sophus/se3.hpp"
#include "sophus/sim3.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <boost/uuid/uuid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/msg/sim3_transform_stamped.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>

class System;

class OrbSlam3Wrapper : public rclcpp::Node {
public:
  OrbSlam3Wrapper(string node_name, string voc_file, ORB_SLAM3::System::eSensor sensor_type);

protected:
  uint agentId;

  string settings_file;

  map<uint, Peer*> connectedPeers;

  bool isLostFromBaseMap = false;
  ORB_SLAM3::Map* baseMap;

  ORB_SLAM3::System* pSLAM;
  ORB_SLAM3::System::eSensor sensor_type;

  rclcpp::Node::SharedPtr node_handle_;

  std::mutex mutexWrapper;

  PublishRosVizTopics* publishRosVizTopics;
  ReferenceFrameManager* referenceFrameManager;

  // ROS services
  rclcpp::Service<interfaces::srv::GetCurrentMap>::SharedPtr get_current_map_service;
  rclcpp::Service<interfaces::srv::GetMapPoints>::SharedPtr getMapPointsService;

  // ROS subscriptions
  rclcpp::Subscription<interfaces::msg::NewKeyFrames>::SharedPtr newKeyFramesSub;
  rclcpp::Subscription<interfaces::msg::NewKeyFrameBows>::SharedPtr newKeyFrameBowsSub;
  rclcpp::Subscription<interfaces::msg::SuccessfullyMerged>::SharedPtr successfullyMergedSub;
  rclcpp::Subscription<interfaces::msg::IsLostFromBaseMap>::SharedPtr isLostFromBaseMapSub;
  rclcpp::Subscription<interfaces::msg::LoopClosureTriggers>::SharedPtr loopClosureTriggersSub;

  rclcpp::TimerBase::SharedPtr updateScaleTimer;

  ORB_SLAM3::KeyFrameDatabase* dummyKFDB;

  bool newFrameProcessed;
  rclcpp::Time lastFrameTimestamp;

  void run();

  void handleGetCurrentMapRequest(const std::shared_ptr<interfaces::srv::GetCurrentMap::Request> request,
    std::shared_ptr<interfaces::srv::GetCurrentMap::Response> response);
  void handleGetCurrentMapResponse(rclcpp::Client<interfaces::srv::GetCurrentMap>::SharedFuture future);

  void sendNewKeyFrames();
  void receiveNewKeyFrames(const interfaces::msg::NewKeyFrames::SharedPtr msg);

  void sendNewKeyFrameBows();
  void receiveNewKeyFrameBows(const interfaces::msg::NewKeyFrameBows::SharedPtr msg);

  void updateSuccessfullyMerged();
  void receiveSuccessfullyMergedMsg(const interfaces::msg::SuccessfullyMerged::SharedPtr msg);

  void updateIsLostFromBaseMap();
  void receiveIsLostFromBaseMapMsg(const interfaces::msg::IsLostFromBaseMap::SharedPtr msg);

  void updateMapScale();

  void handleGetMapPointsRequest(const std::shared_ptr<interfaces::srv::GetMapPoints::Request> request,
    std::shared_ptr<interfaces::srv::GetMapPoints::Response> response);

  void sendLoopClosureTriggers();
  void receiveLoopClosureTriggers(const interfaces::msg::LoopClosureTriggers::SharedPtr msg);

  boost::uuids::uuid arrayToUuid(array<unsigned char, 16> array);
  array<unsigned char, 16> uuidToArray(boost::uuids::uuid uuid);

  unique_ptr<ORB_SLAM3::Map> deepCopyMap(ORB_SLAM3::Map* targetMap);

  // Publish data to ros topics
  void publish_topics(rclcpp::Time msg_time, Eigen::Vector3f Wbb = Eigen::Vector3f::Zero());

  sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(
    std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time);

  tuple<Sophus::SE3f, float> ransacPointSetAlignment(vector<Eigen::Vector3f> sourcePoints,
    vector<Eigen::Vector3f> targetPoints, int numSamples, int iterations, float inlierThreshold);

  tuple<Sophus::SE3f, float> pointSetAlignment(
    vector<Eigen::Vector3f> sourcePoints, vector<Eigen::Vector3f> targetPoints);
};