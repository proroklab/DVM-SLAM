#pragma once

#include "KeyFrame.h"
#include "Map.h"
#include "System.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "interfaces/msg/is_lost_from_base_map.hpp"
#include "interfaces/msg/new_key_frame_bows.hpp"
#include "interfaces/msg/new_key_frames.hpp"
#include "interfaces/msg/successfully_merged.hpp"
#include "interfaces/msg/uuid.hpp"
#include "interfaces/srv/get_current_map.hpp"
#include "interfaces/srv/get_map_points.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "peer.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sophus/se3.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <boost/uuid/uuid.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

class System;

class OrbSlam3Wrapper : public rclcpp::Node {
public:
  OrbSlam3Wrapper(string node_name, string voc_file, string settings_file, ORB_SLAM3::System::eSensor sensor_type);

protected:
  uint agentId;

  map<uint, Peer*> connectedPeers;

  bool isLostFromBaseMap = false;
  ORB_SLAM3::Map* baseMap;

  ORB_SLAM3::System* pSLAM;
  ORB_SLAM3::System::eSensor sensor_type;

  rclcpp::Node::SharedPtr node_handle_;

  image_transport::ImageTransport image_transport;

  std::mutex mutexWrapper;

  // ROS publishers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_mappoints_pub;
  image_transport::Publisher tracking_img_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr kf_markers_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

  // ROS services
  rclcpp::Service<interfaces::srv::GetCurrentMap>::SharedPtr get_current_map_service;
  rclcpp::Service<interfaces::srv::GetMapPoints>::SharedPtr getMapPointsService;

  // ROS subscriptions
  rclcpp::Subscription<interfaces::msg::NewKeyFrames>::SharedPtr newKeyFramesSub;
  rclcpp::Subscription<interfaces::msg::NewKeyFrameBows>::SharedPtr newKeyFrameBowsSub;
  rclcpp::Subscription<interfaces::msg::SuccessfullyMerged>::SharedPtr successfullyMergedSub;
  rclcpp::Subscription<interfaces::msg::IsLostFromBaseMap>::SharedPtr isLostFromBaseMapSub;

  rclcpp::TimerBase::SharedPtr shareNewKeyFrameBowsTimer;
  rclcpp::TimerBase::SharedPtr shareNewKeyFramesTimer;
  rclcpp::TimerBase::SharedPtr updateMapScaleTimer;

  string world_frame_id = "/world";
  string origin_frame_id = "/origin";
  string imu_frame_id = "/imu";
  string cam_frame_id = "camera";

  vector<geometry_msgs::msg::Point> cameraWireframe;

  ORB_SLAM3::KeyFrameDatabase* dummyKFDB;

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

  boost::uuids::uuid arrayToUuid(array<unsigned char, 16> array);
  array<unsigned char, 16> uuidToArray(boost::uuids::uuid uuid);

  unique_ptr<ORB_SLAM3::Map> deepCopyMap(ORB_SLAM3::Map* targetMap);

  void processedNewFrame(rclcpp::Time msg_time);

  // Publish data to ros topics
  void publish_topics(rclcpp::Time msg_time, Eigen::Vector3f Wbb = Eigen::Vector3f::Zero());
  void publish_camera_pose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msg_time);
  void publish_tf_transform(const Sophus::SE3f& T_SE3f, const std::string& frame_id, const std::string& child_frame_id,
    const rclcpp::Time& msg_time);
  void publish_tracking_img(cv::Mat image, rclcpp::Time msg_time);
  void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint*> tracked_points, rclcpp::Time msg_time);
  void publish_all_points(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time);
  void publish_keyframes(std::vector<ORB_SLAM3::KeyFrame*> keyFrames, rclcpp::Time msg_time);
  void publish_body_odom(
    Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body, rclcpp::Time msg_time);

  void resetVisualization();

  sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(
    std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time);

  tuple<Sophus::SE3f, float> pointSetAlignment(
    vector<Eigen::Vector3f> sourcePoints, vector<Eigen::Vector3f> targetPoints);
};