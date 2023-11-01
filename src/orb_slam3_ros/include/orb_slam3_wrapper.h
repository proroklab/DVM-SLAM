#include "System.h"
#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "image_transport/image_transport.hpp"
#include "interfaces/srv/add_map.hpp"
#include "interfaces/srv/get_current_map.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/transform_broadcaster.h"
#include "visualization_msgs/msg/marker.hpp"
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

#pragma once

using namespace std;

class OrbSlam3Wrapper : public rclcpp::Node {
public:
  OrbSlam3Wrapper(string node_name, string voc_file, string settings_file,
                  ORB_SLAM3::System::eSensor sensor_type);

protected:
  string robot_name;

  ORB_SLAM3::System *pSLAM;
  ORB_SLAM3::System::eSensor sensor_type;

  rclcpp::Node::SharedPtr node_handle_;

  image_transport::ImageTransport image_transport;

  // ROS publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
      tracked_mappoints_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_mappoints_pub;
  image_transport::Publisher tracking_img_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr kf_markers_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

  // ROS services
  rclcpp::Service<interfaces::srv::GetCurrentMap>::SharedPtr
      get_current_map_service;
  rclcpp::Service<interfaces::srv::AddMap>::SharedPtr add_map_service;

  string world_frame_id = "/world";
  string imu_frame_id = "/imu";
  string cam_frame_id = "camera";

  void getCurrentMap(
      const std::shared_ptr<interfaces::srv::GetCurrentMap::Request> request,
      std::shared_ptr<interfaces::srv::GetCurrentMap::Response> response);

  void addMap(const std::shared_ptr<interfaces::srv::AddMap::Request> request,
              std::shared_ptr<interfaces::srv::AddMap::Response> response);

  void publish_topics(rclcpp::Time msg_time,
                      Eigen::Vector3f Wbb = Eigen::Vector3f::Zero());

  void publish_camera_pose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msg_time);

  void publish_tf_transform(const Sophus::SE3f &T_SE3f,
                            const std::string &frame_id,
                            const std::string &child_frame_id,
                            const rclcpp::Time &msg_time);

  void publish_tracking_img(cv::Mat image, rclcpp::Time msg_time);

  void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint *> tracked_points,
                              rclcpp::Time msg_time);

  void publish_all_points(std::vector<ORB_SLAM3::MapPoint *> map_points,
                          rclcpp::Time msg_time);

  void publish_kf_markers(std::vector<Sophus::SE3f> vKFposes,
                          rclcpp::Time msg_time);

  void publish_body_odom(Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f,
                         Eigen::Vector3f ang_vel_body, rclcpp::Time msg_time);

  sensor_msgs::msg::PointCloud2
  mappoint_to_pointcloud(std::vector<ORB_SLAM3::MapPoint *> map_points,
                         rclcpp::Time msg_time);
};