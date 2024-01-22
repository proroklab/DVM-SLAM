#include "orb_slam3_wrapper.h"

using namespace std;

OrbSlam3Wrapper::OrbSlam3Wrapper(
  string node_name, string voc_file, string settings_file, ORB_SLAM3::System::eSensor sensor_type)
  : Node("robot" + to_string(agentId))
  , node_handle_(std::shared_ptr<OrbSlam3Wrapper>(this, [](auto*) {}))
  , image_transport(node_handle_) {
  // Create parameters
  this->declare_parameter("agentId", 1);
  agentId = this->get_parameter("agentId").as_int();

  node_name = "robot" + to_string(agentId);
  cout << node_name << endl;

  this->sensor_type = sensor_type;
  pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, true);

  // Create publishers
  pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(node_name + "/camera_pose", 1);
  tracked_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_points", 1);
  all_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/all_points", 1);
  tracking_img_pub = image_transport.advertise(node_name + "/tracking_image", 1);
  kf_markers_pub = this->create_publisher<visualization_msgs::msg::Marker>(node_name + "/kf_markers", 1000);
  if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO
    || sensor_type == ORB_SLAM3::System::IMU_RGBD) {
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(node_name + "/body_odom", 1);
  }

  // Create services
  get_current_map_service = this->create_service<interfaces::srv::GetCurrentMap>(node_name + "/get_current_map",
    std::bind(&OrbSlam3Wrapper::getCurrentMap, this, std::placeholders::_1, std::placeholders::_2));
  add_map_service = this->create_service<interfaces::srv::AddMap>(
    node_name + "/add_map", std::bind(&OrbSlam3Wrapper::addMap, this, std::placeholders::_1, std::placeholders::_2));
};

void OrbSlam3Wrapper::getCurrentMap(const std::shared_ptr<interfaces::srv::GetCurrentMap::Request> request,
  std::shared_ptr<interfaces::srv::GetCurrentMap::Response> response) {
  response->serialized_map = pSLAM->GetSerializedCurrentMap();
}

void OrbSlam3Wrapper::addMap(const std::shared_ptr<interfaces::srv::AddMap::Request> request,
  std::shared_ptr<interfaces::srv::AddMap::Response> response) {

  RCLCPP_INFO(this->get_logger(), "Received serialized map. Size: %d", request->serialized_map.size());

  pSLAM->AddSerializedMap(request->serialized_map);
}

void OrbSlam3Wrapper::publish_topics(rclcpp::Time msg_time, Eigen::Vector3f Wbb) {
  Sophus::SE3f Twc = pSLAM->GetCamTwc();

  if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0)) // avoid publishing NaN
    return;

  // Common topics
  publish_camera_pose(Twc, msg_time);
  publish_tf_transform(Twc, world_frame_id, cam_frame_id, msg_time);

  publish_tracking_img(pSLAM->GetCurrentFrame(), msg_time);
  publish_tracked_points(pSLAM->GetTrackedMapPoints(), msg_time);
  publish_all_points(pSLAM->GetAllMapPoints(), msg_time);
  publish_kf_markers(pSLAM->GetAllKeyframePoses(), msg_time);

  // IMU-specific topics
  if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO
    || sensor_type == ORB_SLAM3::System::IMU_RGBD) {
    // Body pose and translational velocity can be obtained from ORB-SLAM3
    Sophus::SE3f Twb = pSLAM->GetImuTwb();
    Eigen::Vector3f Vwb = pSLAM->GetImuVwb();

    // IMU provides body angular velocity in body frame (Wbb) which is
    // transformed to world frame (Wwb)
    Sophus::Matrix3f Rwb = Twb.rotationMatrix();
    Eigen::Vector3f Wwb = Rwb * Wbb;

    publish_tf_transform(Twb, world_frame_id, imu_frame_id, msg_time);
    publish_body_odom(Twb, Vwb, Wwb, msg_time);
  }
}

void OrbSlam3Wrapper::publish_camera_pose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msg_time) {
  geometry_msgs::msg::PoseStamped pose_msg;
  pose_msg.header.frame_id = world_frame_id;
  pose_msg.header.stamp = msg_time;

  pose_msg.pose.position.x = Tcw_SE3f.translation().x();
  pose_msg.pose.position.y = Tcw_SE3f.translation().y();
  pose_msg.pose.position.z = Tcw_SE3f.translation().z();

  pose_msg.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
  pose_msg.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
  pose_msg.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
  pose_msg.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

  pose_pub->publish(pose_msg);
}

void OrbSlam3Wrapper::publish_tf_transform(const Sophus::SE3f& T_SE3f, const std::string& frame_id,
  const std::string& child_frame_id, const rclcpp::Time& msg_time) {
  static tf2_ros::TransformBroadcaster tf_broadcaster(*this);

  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = msg_time;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;

  auto translation = T_SE3f.translation();
  auto quaternion = T_SE3f.unit_quaternion();

  transform_stamped.transform.translation.x = translation[0];
  transform_stamped.transform.translation.y = translation[1];
  transform_stamped.transform.translation.z = translation[2];
  transform_stamped.transform.rotation.x = quaternion.x();
  transform_stamped.transform.rotation.y = quaternion.y();
  transform_stamped.transform.rotation.z = quaternion.z();
  transform_stamped.transform.rotation.w = quaternion.w();

  tf_broadcaster.sendTransform(transform_stamped);
}

void OrbSlam3Wrapper::publish_tracking_img(cv::Mat image, rclcpp::Time msg_time) {
  std_msgs::msg::Header header;
  header.stamp = msg_time;
  header.frame_id = world_frame_id;

  const sensor_msgs::msg::Image::SharedPtr rendered_image_msg = cv_bridge::CvImage(header, "bgr8", image).toImageMsg();

  tracking_img_pub.publish(rendered_image_msg);
}

void OrbSlam3Wrapper::publish_tracked_points(std::vector<ORB_SLAM3::MapPoint*> tracked_points, rclcpp::Time msg_time) {
  sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time);

  tracked_mappoints_pub->publish(cloud);
}

void OrbSlam3Wrapper::publish_all_points(std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time) {
  sensor_msgs::msg::PointCloud2 cloud = mappoint_to_pointcloud(map_points, msg_time);

  all_mappoints_pub->publish(cloud);
}

void OrbSlam3Wrapper::publish_kf_markers(std::vector<Sophus::SE3f> vKFposes, rclcpp::Time msg_time) {
  int numKFs = vKFposes.size();
  if (numKFs == 0)
    return;

  visualization_msgs::msg::Marker kf_markers;
  kf_markers.header.frame_id = world_frame_id;
  kf_markers.ns = "kf_markers";
  kf_markers.type = visualization_msgs::msg::Marker::SPHERE_LIST;
  kf_markers.action = visualization_msgs::msg::Marker::ADD;
  kf_markers.pose.orientation.w = 1.0;
  kf_markers.lifetime = rclcpp::Duration(0, 0); // not sure?

  kf_markers.id = 0;
  kf_markers.scale.x = 0.05;
  kf_markers.scale.y = 0.05;
  kf_markers.scale.z = 0.05;
  kf_markers.color.g = 1.0;
  kf_markers.color.a = 1.0;

  for (int i = 0; i <= numKFs; i++) {
    geometry_msgs::msg::Point kf_marker;
    kf_marker.x = vKFposes[i].translation().x();
    kf_marker.y = vKFposes[i].translation().y();
    kf_marker.z = vKFposes[i].translation().z();
    kf_markers.points.push_back(kf_marker);
  }

  kf_markers_pub->publish(kf_markers);
}

void OrbSlam3Wrapper::publish_body_odom(
  Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body, rclcpp::Time msg_time) {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.child_frame_id = imu_frame_id;
  odom_msg.header.frame_id = world_frame_id;
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

sensor_msgs::msg::PointCloud2 OrbSlam3Wrapper::mappoint_to_pointcloud(
  std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time) {
  const int num_channels = 3; // x y z

  if (map_points.size() == 0) {
    std::cout << "Map point vector is empty!" << std::endl;
  }

  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.stamp = msg_time;
  cloud.header.frame_id = world_frame_id;
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