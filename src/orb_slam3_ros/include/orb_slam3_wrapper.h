#pragma once

#include "Map.h"
#include "System.h"
#include "opencv2/highgui/highgui.hpp"
#include "peer.h"
#include "publish_ros_viz_topics.h"
#include "sophus/se3.hpp"
#include <boost/uuid/uuid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/msg/change_coordinate_frame.hpp>
#include <interfaces/msg/map_to_attempt_merge.hpp>
#include <interfaces/msg/sim3_transform_stamped.hpp>
#include <interfaces/msg/successfully_merged.hpp>
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

  // ROS broadcast publishers
  rclcpp::Publisher<interfaces::msg::SuccessfullyMerged>::SharedPtr successfullyMergedPub;

  // ROS services
  rclcpp::Service<interfaces::srv::GetCurrentMap>::SharedPtr get_current_map_service;
  rclcpp::Service<interfaces::srv::GetMapPoints>::SharedPtr getMapPointsService;

  // ROS subscriptions
  rclcpp::Subscription<interfaces::msg::NewKeyFrames>::SharedPtr newKeyFramesSub;
  rclcpp::Subscription<interfaces::msg::NewKeyFrameBows>::SharedPtr newKeyFrameBowsSub;
  rclcpp::Subscription<interfaces::msg::SuccessfullyMerged>::SharedPtr successfullyMergedSub;
  rclcpp::Subscription<interfaces::msg::IsLostFromBaseMap>::SharedPtr isLostFromBaseMapSub;
  rclcpp::Subscription<interfaces::msg::LoopClosureTriggers>::SharedPtr loopClosureTriggersSub;
  rclcpp::Subscription<interfaces::msg::ChangeCoordinateFrame>::SharedPtr changeCoordinateFrameSub;
  rclcpp::Subscription<interfaces::msg::MapToAttemptMerge>::SharedPtr mapToAttemptMergeSub;

  rclcpp::TimerBase::SharedPtr updateScaleTimer;

  ORB_SLAM3::KeyFrameDatabase* dummyKFDB;

  bool newFrameProcessed;
  rclcpp::Time lastFrameTimestamp;

  void run();

  void handleGetCurrentMapRequest(const std::shared_ptr<interfaces::srv::GetCurrentMap::Request> request,
    std::shared_ptr<interfaces::srv::GetCurrentMap::Response> response);
  void handleGetCurrentMapResponse(rclcpp::Client<interfaces::srv::GetCurrentMap>::SharedFuture future);

  void receiveMapToAttemptMerge(interfaces::msg::MapToAttemptMerge::SharedPtr msg);

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

  void sendChangeCoordinateFrame(uint parentAgentId, Sophus::Sim3d parentToCurrentTransform);
  void receiveChangeCoordinateFrame(const interfaces::msg::ChangeCoordinateFrame::SharedPtr msg);

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

  bool isSuccessfullyMerged(uint otherAgentId);
  vector<Peer*> getSuccessfullyMergedPeers();
  bool isLeadNodeInGroup();
};