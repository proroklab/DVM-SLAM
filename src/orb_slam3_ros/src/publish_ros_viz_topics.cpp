#include "publish_ros_viz_topics.h"
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <rclcpp/node.hpp>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

#define BEST_EFFORT_QOS rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data)
#define RELIABLE_QOS rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_services_default)

PublishRosVizTopics::PublishRosVizTopics(shared_ptr<rclcpp::Node> node, string node_name,
  ORB_SLAM3::System::eSensor sensor_type, uint agentId, ReferenceFrameManager* referenceFrameManager)
  : node(node)
  , image_transport(node)
  , agentId(agentId)
  , referenceFrameManager(referenceFrameManager) {
  // Create publishers
  pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(node_name + "/camera_pose", RELIABLE_QOS);
  pose_marker_pub
    = node->create_publisher<visualization_msgs::msg::Marker>(node_name + "/camera_pose_marker", BEST_EFFORT_QOS);
  tracked_mappoints_pub
    = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_points", BEST_EFFORT_QOS);
  all_mappoints_pub = node->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/all_points", BEST_EFFORT_QOS);
  tracking_img_pub = image_transport.advertise(node_name + "/tracking_image", 10);
  kf_markers_pub
    = node->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/kf_markers", BEST_EFFORT_QOS);
  if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO
    || sensor_type == ORB_SLAM3::System::IMU_RGBD) {
    odom_pub = node->create_publisher<nav_msgs::msg::Odometry>(node_name + "/body_odom", BEST_EFFORT_QOS);
  }
  sim3_transform_pub
    = node->create_publisher<interfaces::msg::Sim3TransformStamped>("/sim3_transform", BEST_EFFORT_QOS);

  // Create camera wireframe
  float scaleX = 0.05;
  float scaleY = 0.03;
  float scaleZ = 0.05;
  float cameraWireframePoints[][3] = {
    { 0, 0, 0 },
    { 1, 1, 1 },
    { 0, 0, 0 },
    { 1, -1, 1 },
    { 0, 0, 0 },
    { -1, 1, 1 },
    { 0, 0, 0 },
    { -1, -1, 1 },
    { 1, 1, 1 },
    { 1, -1, 1 },
    { 1, -1, 1 },
    { -1, -1, 1 },
    { -1, -1, 1 },
    { -1, 1, 1 },
    { -1, 1, 1 },
    { 1, 1, 1 },
  };
  for (float* cameraWireframePoint : cameraWireframePoints) {
    geometry_msgs::msg::Point point;
    point.x = cameraWireframePoint[0] * scaleX;
    point.y = cameraWireframePoint[1] * scaleY;
    point.z = cameraWireframePoint[2] * scaleZ;
    cameraWireframe.push_back(point);
  }
}

void PublishRosVizTopics::publish_camera_pose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msg_time) {
  visualization_msgs::msg::Marker cameraPoseWireframe;
  cameraPoseWireframe.header.frame_id = referenceFrameManager->slam_system_frame_id;
  cameraPoseWireframe.header.stamp = msg_time;
  cameraPoseWireframe.ns = "cameraPoseWireframe";
  cameraPoseWireframe.type = visualization_msgs::msg::Marker::LINE_LIST;
  cameraPoseWireframe.action = visualization_msgs::msg::Marker::ADD;

  cameraPoseWireframe.id = 0;
  cameraPoseWireframe.scale.x = 0.005;
  cameraPoseWireframe.scale.y = 0.005;
  cameraPoseWireframe.scale.z = 0.005;
  cameraPoseWireframe.color.g = 1.0;
  cameraPoseWireframe.color.a = 1.0;

  cameraPoseWireframe.pose.position.x = Tcw_SE3f.translation().x();
  cameraPoseWireframe.pose.position.y = Tcw_SE3f.translation().y();
  cameraPoseWireframe.pose.position.z = Tcw_SE3f.translation().z();

  cameraPoseWireframe.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
  cameraPoseWireframe.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
  cameraPoseWireframe.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
  cameraPoseWireframe.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

  cameraPoseWireframe.points = cameraWireframe;

  pose_marker_pub->publish(cameraPoseWireframe);

  geometry_msgs::msg::PoseStamped poseStampedMsg;
  poseStampedMsg.header.stamp = msg_time;
  poseStampedMsg.header.frame_id = referenceFrameManager->slam_system_frame_id;
  poseStampedMsg.pose.position.x = Tcw_SE3f.translation().x();
  poseStampedMsg.pose.position.y = Tcw_SE3f.translation().y();
  poseStampedMsg.pose.position.z = Tcw_SE3f.translation().z();
  poseStampedMsg.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
  poseStampedMsg.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
  poseStampedMsg.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
  poseStampedMsg.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

  pose_pub->publish(poseStampedMsg);
}

void PublishRosVizTopics::publish_tf_transform(const Sophus::SE3d& T_SE3d, const std::string& frame_id,
  const std::string& child_frame_id, const rclcpp::Time& msg_time) {
  static tf2_ros::TransformBroadcaster tf_broadcaster(node);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = msg_time;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;

  auto translation = T_SE3d.translation();
  auto quaternion = T_SE3d.unit_quaternion();

  transform_stamped.transform.translation.x = translation[0];
  transform_stamped.transform.translation.y = translation[1];
  transform_stamped.transform.translation.z = translation[2];
  transform_stamped.transform.rotation.x = quaternion.x();
  transform_stamped.transform.rotation.y = quaternion.y();
  transform_stamped.transform.rotation.z = quaternion.z();
  transform_stamped.transform.rotation.w = quaternion.w();

  tf_broadcaster.sendTransform(transform_stamped);
}

void PublishRosVizTopics::publish_sim3_transform(const Sophus::Sim3d& T_Sim3d, const std::string& frame_id,
  const std::string& child_frame_id, const rclcpp::Time& msg_time) {
  interfaces::msg::Sim3TransformStamped sim3_transform_stamped;
  sim3_transform_stamped.header.stamp = msg_time;
  sim3_transform_stamped.header.frame_id = frame_id;
  sim3_transform_stamped.child_frame_id = child_frame_id;

  auto translation = T_Sim3d.translation();
  auto quaternion = T_Sim3d.quaternion();

  sim3_transform_stamped.transform.translation.x = translation[0];
  sim3_transform_stamped.transform.translation.y = translation[1];
  sim3_transform_stamped.transform.translation.z = translation[2];
  sim3_transform_stamped.transform.rotation.x = quaternion.x();
  sim3_transform_stamped.transform.rotation.y = quaternion.y();
  sim3_transform_stamped.transform.rotation.z = quaternion.z();
  sim3_transform_stamped.transform.rotation.w = quaternion.w();
  sim3_transform_stamped.transform.scale = T_Sim3d.scale();

  sim3_transform_pub->publish(sim3_transform_stamped);
}

void PublishRosVizTopics::publish_tracking_img(cv::Mat image, rclcpp::Time msg_time) {
  std_msgs::msg::Header header;
  header.stamp = msg_time;
  header.frame_id = referenceFrameManager->cam_frame_id;

  const sensor_msgs::msg::Image::SharedPtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

  tracking_img_pub.publish(rendered_image_msg);
}

void PublishRosVizTopics::publish_tracked_points(
  std::vector<ORB_SLAM3::MapPoint*> tracked_points, rclcpp::Time msg_time) {
  sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time);

  tracked_mappoints_pub->publish(cloud);
}

void PublishRosVizTopics::publish_all_points(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time) {
  sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(map_points, msg_time);

  all_mappoints_pub->publish(cloud);
}

void PublishRosVizTopics::publish_keyframes(std::vector<ORB_SLAM3::KeyFrame*> keyFrames, rclcpp::Time msg_time) {
  visualization_msgs::msg::MarkerArray keyFrameMarkers;

  // Create keyframe wireframes
  for (ORB_SLAM3::KeyFrame* keyFrame : keyFrames) {
    if (keyFrame->isBad())
      continue;

    visualization_msgs::msg::Marker keyFrameWireframe;
    keyFrameWireframe.header.frame_id = referenceFrameManager->slam_system_frame_id;
    keyFrameWireframe.ns = "keyFrameWireframes";
    keyFrameWireframe.type = visualization_msgs::msg::Marker::LINE_LIST;
    keyFrameWireframe.action = visualization_msgs::msg::Marker::ADD;

    keyFrameWireframe.id = keyFrame->mnId;
    keyFrameWireframe.scale.x = 0.005;
    keyFrameWireframe.scale.y = 0.005;
    keyFrameWireframe.scale.z = 0.005;
    keyFrameWireframe.color.a = 1.0;

    if (keyFrame->creatorAgentId == agentId)
      keyFrameWireframe.color.g = 0.5;
    else
      keyFrameWireframe.color.r = 0.5;

    Sophus::SE3f worldToKeyFrame = keyFrame->GetPoseInverse();

    keyFrameWireframe.pose.position.x = worldToKeyFrame.translation().x();
    keyFrameWireframe.pose.position.y = worldToKeyFrame.translation().y();
    keyFrameWireframe.pose.position.z = worldToKeyFrame.translation().z();
    keyFrameWireframe.pose.orientation.w = worldToKeyFrame.unit_quaternion().coeffs().w();
    keyFrameWireframe.pose.orientation.x = worldToKeyFrame.unit_quaternion().coeffs().x();
    keyFrameWireframe.pose.orientation.y = worldToKeyFrame.unit_quaternion().coeffs().y();
    keyFrameWireframe.pose.orientation.z = worldToKeyFrame.unit_quaternion().coeffs().z();

    keyFrameWireframe.points = cameraWireframe;

    keyFrameMarkers.markers.push_back(keyFrameWireframe);
  }

  // Create covisibility graph
  for (ORB_SLAM3::KeyFrame* keyFrame : keyFrames) {
    if (keyFrame->isBad())
      continue;

    vector<ORB_SLAM3::KeyFrame*> connectedKeyFrames = keyFrame->GetCovisiblesByWeight(100);

    visualization_msgs::msg::Marker connectedKeyFrameLines;
    connectedKeyFrameLines.header.frame_id = referenceFrameManager->slam_system_frame_id;
    connectedKeyFrameLines.ns = "connectedKeyFrameLines";
    connectedKeyFrameLines.type = visualization_msgs::msg::Marker::LINE_LIST;
    connectedKeyFrameLines.action = visualization_msgs::msg::Marker::ADD;

    connectedKeyFrameLines.id = keyFrame->mnId;
    connectedKeyFrameLines.scale.x = 0.0025;
    connectedKeyFrameLines.scale.y = 0.0025;
    connectedKeyFrameLines.scale.z = 0.0025;
    connectedKeyFrameLines.color.b = 1.0;
    connectedKeyFrameLines.color.a = 1.0;

    for (ORB_SLAM3::KeyFrame* connectedKeyFrame : connectedKeyFrames) {
      if (connectedKeyFrame->isBad() || connectedKeyFrame->mnId < keyFrame->mnId)
        continue;

      geometry_msgs::msg::Point keyFramePoint;
      keyFramePoint.x = keyFrame->GetCameraCenter().x();
      keyFramePoint.y = keyFrame->GetCameraCenter().y();
      keyFramePoint.z = keyFrame->GetCameraCenter().z();
      connectedKeyFrameLines.points.push_back(keyFramePoint);

      geometry_msgs::msg::Point connectedKeyFramePoint;
      connectedKeyFramePoint.x = connectedKeyFrame->GetCameraCenter().x();
      connectedKeyFramePoint.y = connectedKeyFrame->GetCameraCenter().y();
      connectedKeyFramePoint.z = connectedKeyFrame->GetCameraCenter().z();
      connectedKeyFrameLines.points.push_back(connectedKeyFramePoint);
    }

    if (connectedKeyFrameLines.points.size() != 0) {
      keyFrameMarkers.markers.push_back(connectedKeyFrameLines);
    }
  }

  kf_markers_pub->publish(keyFrameMarkers);
}

// void PublishRosVizTopics::publish_kf_markers(std::vector<Sophus::SE3f> vKFposes, rclcpp::Time msg_time) {

void PublishRosVizTopics::publish_body_odom(
  Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body, rclcpp::Time msg_time) {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.child_frame_id = referenceFrameManager->imu_frame_id;
  odom_msg.header.frame_id = referenceFrameManager->slam_system_frame_id;
  odom_msg.header.stamp = msg_time;

  odom_msg.pose.pose.position.x = Twb_SE3f.translation().x();
  odom_msg.pose.pose.position.y = Twb_SE3f.translation().y();
  odom_msg.pose.pose.position.z = Twb_SE3f.translation().z();

  odom_msg.pose.pose.orientation.w = Twb_SE3f.unit_quaternion().coeffs().w();
  odom_msg.pose.pose.orientation.x = Twb_SE3f.unit_quaternion().coeffs().x();
  odom_msg.pose.pose.orientation.y = Twb_SE3f.unit_quaternion().coeffs().y();
  odom_msg.pose.pose.orientation.z = Twb_SE3f.unit_quaternion().coeffs().z();

  odom_msg.twist.twist.linear.x = Vwb_E3f.x();
  odom_msg.twist.twist.linear.y = Vwb_E3f.y();
  odom_msg.twist.twist.linear.z = Vwb_E3f.z();

  odom_msg.twist.twist.angular.x = ang_vel_body.x();
  odom_msg.twist.twist.angular.y = ang_vel_body.y();
  odom_msg.twist.twist.angular.z = ang_vel_body.z();

  odom_pub->publish(odom_msg);
}

void PublishRosVizTopics::resetVisualization() {

  // Delete all keyframe markers
  visualization_msgs::msg::MarkerArray kf_markers;
  visualization_msgs::msg::Marker kf_marker;

  kf_marker.ns = "keyFrameWireframes";
  kf_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  kf_markers.markers.push_back(kf_marker);
  kf_marker.ns = "connectedKeyFrameLines";
  kf_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  kf_markers.markers.push_back(kf_marker);

  kf_markers_pub->publish(kf_markers);
}

sensor_msgs::msg::PointCloud2 PublishRosVizTopics::mappoint_to_pointcloud(
  std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time) {
  const int num_channels = 3; // x y z

  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.stamp = msg_time;
  cloud.header.frame_id = referenceFrameManager->slam_system_frame_id;
  cloud.height = 1;
  cloud.width = map_points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;
  cloud.point_step = num_channels * sizeof(float);
  cloud.row_step = cloud.point_step * cloud.width;
  cloud.fields.resize(num_channels);

  std::string channel_id[] = { "x", "y", "z" };

  for (int i = 0; i < num_channels; i++) {
    cloud.fields[i].name = channel_id[i];
    cloud.fields[i].offset = i * sizeof(float);
    cloud.fields[i].count = 1;
    cloud.fields[i].datatype = sensor_msgs::msg::PointField::FLOAT32;
  }

  cloud.data.resize(cloud.row_step * cloud.height);

  unsigned char* cloud_data_ptr = &(cloud.data[0]);

  for (unsigned int i = 0; i < cloud.width; i++) {
    if (map_points[i]) {
      Eigen::Vector3d P3Dw = map_points[i]->GetWorldPos().cast<double>();

      tf2::Vector3 point_translation(P3Dw.x(), P3Dw.y(), P3Dw.z());

      float data_array[num_channels] = { static_cast<float>(point_translation.x()),
        static_cast<float>(point_translation.y()), static_cast<float>(point_translation.z()) };

      memcpy(cloud_data_ptr + (i * cloud.point_step), data_array, num_channels * sizeof(float));
    }
  }
  return cloud;
}