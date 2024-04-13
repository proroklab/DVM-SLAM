#include "KeyFrame.h"
#include "MapPoint.h"
#include "System.h"
#include "image_transport/image_transport.hpp"
#include "rclcpp/publisher.hpp"
#include "reference_frame_manager.h"
#include "sophus/se3.hpp"
#include "sophus/sim3.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <boost/uuid/uuid.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/msg/sim3_transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

class PublishRosVizTopics {
public:
  PublishRosVizTopics(shared_ptr<rclcpp::Node> node, string node_name, ORB_SLAM3::System::eSensor sensor_type,
    uint agentId, ReferenceFrameManager* referenceFrameManager);

  void publish_topics(rclcpp::Time msg_time, Eigen::Vector3f Wbb);

  void publish_camera_pose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msg_time);

  void publish_tf_transform(const Sophus::SE3d& T_SE3d, const std::string& frame_id, const std::string& child_frame_id,
    const rclcpp::Time& msg_time);

  void publish_sim3_transform(const Sophus::Sim3d& T_Sim3d, const std::string& frame_id,
    const std::string& child_frame_id, const rclcpp::Time& msg_time);

  void publish_tracking_img(cv::Mat image, rclcpp::Time msg_time);

  void publish_tracked_points(std::vector<ORB_SLAM3::MapPoint*> tracked_points, rclcpp::Time msg_time);

  void publish_all_points(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time);

  void publish_keyframes(std::vector<ORB_SLAM3::KeyFrame*> keyFrames, rclcpp::Time msg_time);

  void publish_body_odom(
    Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body, rclcpp::Time msg_time);

  void resetVisualization();

protected:
  image_transport::ImageTransport image_transport;

  vector<geometry_msgs::msg::Point> cameraWireframe;

  uint agentId;
  shared_ptr<rclcpp::Node> node;
  ReferenceFrameManager* referenceFrameManager;

  map<boost::uuids::uuid, Sophus::SE3f> last_sent_keyframe_poses;

  // ROS publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pose_marker_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr tracked_mappoints_pub;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr all_mappoints_pub;
  image_transport::Publisher tracking_img_pub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr kf_markers_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<interfaces::msg::Sim3TransformStamped>::SharedPtr sim3_transform_pub;

  sensor_msgs::msg::PointCloud2 mappoint_to_pointcloud(
    std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time);
};