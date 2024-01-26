#include "orb_slam3_wrapper.h"

#include "DBoW2/DBoW2/BowVector.h"
#include "KeyFrame.h"
#include "Map.h"
#include <boost/uuid/uuid_io.hpp>
#include <interfaces/msg/detail/key_frame_bow_vector__struct.hpp>
#include <interfaces/msg/detail/new_key_frames__struct.hpp>
#include <interfaces/msg/detail/uuid__struct.hpp>
#include <interfaces/srv/detail/request_map__struct.hpp>
#include <mutex>
#include <shared_mutex>

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
  pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, agentId, true);

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
    std::bind(&OrbSlam3Wrapper::handleGetCurrentMapRequest, this, std::placeholders::_1, std::placeholders::_2));
  add_map_service = this->create_service<interfaces::srv::AddMap>(node_name + "/add_map",
    std::bind(&OrbSlam3Wrapper::handleAddMapRequest, this, std::placeholders::_1, std::placeholders::_2));
  requestMapService = this->create_service<interfaces::srv::RequestMap>(node_name + "/request_map",
    std::bind(&OrbSlam3Wrapper::handleRequestMapRequest, this, std::placeholders::_1, std::placeholders::_2));

  // Create subscriptions
  newKeyFrameSub = this->create_subscription<interfaces::msg::NewKeyFrames>(
    node_name + "/new_key_frames", 1, std::bind(&OrbSlam3Wrapper::receiveNewKeyFrames, this, std::placeholders::_1));

  if (agentId == 1)
    connectedAgentIds = { 2 };
  else
    connectedAgentIds = { 1 };

  // TODO: create a proper topic handler that handles nodes connecting/disconnecting
  for (uint connectedAgentId : connectedAgentIds) {
    newKeyFramesPubs[connectedAgentId] = this->create_publisher<interfaces::msg::NewKeyFrames>(
      "robot" + to_string(connectedAgentId) + "/new_key_frames", 1);
    requestMapClients[connectedAgentId]
      = this->create_client<interfaces::srv::RequestMap>("robot" + to_string(connectedAgentId) + "/request_map");
  }

  shareNewKeyFramesTimer = this->create_wall_timer(2s, std::bind(&OrbSlam3Wrapper::shareNewKeyFrames, this));
};

void OrbSlam3Wrapper::handleGetCurrentMapRequest(const std::shared_ptr<interfaces::srv::GetCurrentMap::Request> request,
  std::shared_ptr<interfaces::srv::GetCurrentMap::Response> response) {
  unique_lock<mutex> lock(mutexWrapper);

  response->serialized_map = pSLAM->GetSerializedCurrentMap();
}

void OrbSlam3Wrapper::handleAddMapRequest(const std::shared_ptr<interfaces::srv::AddMap::Request> request,
  std::shared_ptr<interfaces::srv::AddMap::Response> response) {
  unique_lock<mutex> lock(mutexWrapper);

  RCLCPP_INFO(this->get_logger(), "Received serialized map. Size: %d", request->serialized_map.size());

  pSLAM->AddSerializedMap(request->serialized_map);
}

void OrbSlam3Wrapper::handleRequestMapRequest(const std::shared_ptr<interfaces::srv::RequestMap::Request> request,
  std::shared_ptr<interfaces::srv::RequestMap::Response> response) {
  unique_lock<mutex> lock(mutexWrapper);

  cout << "Handeling Request Map Request" << endl;

  // Find target keyframe
  keyFrames = pSLAM->GetAllKeyFrames();
  boost::uuids::uuid targetKeyFrameUuid
    = arrayToUuid(request->target_key_frame_uuid); // Uuid of keyframe we want to get the map of
  ORB_SLAM3::KeyFrame* targetKeyFrame = *std::find_if(keyFrames.begin(), keyFrames.end(),
    [targetKeyFrameUuid](const ORB_SLAM3::KeyFrame* element) { return element->uuid == targetKeyFrameUuid; });

  ORB_SLAM3::Map* targetMap = targetKeyFrame->GetMap();

  // Make a deep copy of the target map
  // TODO: make this less terrible!
  ORB_SLAM3::Map* targetMapCopy;

  // HACKY: create dummy objects for post load
  vector<ORB_SLAM3::GeometricCamera*> mvpCameras = pSLAM->GetAtlas()->GetAllCameras();
  set<ORB_SLAM3::GeometricCamera*> dummySCams(mvpCameras.begin(), mvpCameras.end());
  ORB_SLAM3::ORBVocabulary* dummyORBVoc = pSLAM->GetORBVocabulary();
  ORB_SLAM3::KeyFrameDatabase* dummyKFDB = new ORB_SLAM3::KeyFrameDatabase(*dummyORBVoc);
  map<unsigned int, ORB_SLAM3::GeometricCamera*> dummyMCams;
  for (ORB_SLAM3::GeometricCamera* pCam : mvpCameras) {
    dummyMCams[pCam->GetId()] = pCam;
  }

  // Copy using serialization / deserialization
  std::ostringstream oss;
  boost::archive::binary_oarchive oa(oss);
  targetMap->PreSave(dummySCams);
  oa << targetMap;
  std::istringstream iss(oss.str());
  boost::archive::binary_iarchive ia(iss);
  ia >> targetMapCopy;
  targetMapCopy->PostLoad(dummyKFDB, dummyORBVoc, dummyMCams);

  // Remove dontIncludeUuid keyframes
  set<boost::uuids::uuid> dontIncludeUuids;
  for (interfaces::msg::Uuid uuidMsg : request->dont_include_uuids) {
    boost::uuids::uuid dontIncludeUuid = arrayToUuid(uuidMsg.uuid); // Uuid of keyframe we want to get the map of
    dontIncludeUuids.insert(dontIncludeUuid);
  }
  for (ORB_SLAM3::KeyFrame* keyFrame : targetMapCopy->GetAllKeyFrames()) {
    if (dontIncludeUuids.count(keyFrame->uuid) > 0) {
      targetMapCopy->EraseKeyFrame(keyFrame);
    }
  }

  response->serialized_map = pSLAM->SerializeMap(targetMapCopy);
}

void OrbSlam3Wrapper::shareNewKeyFrames() {
  unique_lock<mutex> lock(mutexWrapper);

  keyFrames = pSLAM->GetAllKeyFrames();

  // Send new key frames to all peers
  for (uint connectedAgentId : connectedAgentIds) {
    set<boost::uuids::uuid> newKeyFrameUuids;
    vector<interfaces::msg::KeyFrameBowVector> keyFrameBowVectorMsgs;

    cout << "sent new key frame bows: ";

    for (ORB_SLAM3::KeyFrame* keyFrame : keyFrames) {
      if (keyFrame->creatorAgentId == agentId && sentKeyFrameUuids[connectedAgentId].count(keyFrame->uuid) == 0) {
        interfaces::msg::KeyFrameBowVector keyFrameBowVectorMsg;

        cout << keyFrame->uuid << " ";

        // Set uuid
        std::array<unsigned char, 16> uuidArray = uuidToArray(keyFrame->uuid);
        keyFrameBowVectorMsg.uuid = uuidArray;
        newKeyFrameUuids.insert(keyFrame->uuid);

        // Set bow vector
        vector<int64_t> bowVectorKeys;
        vector<double> bowVectorValues;
        for (auto pair : keyFrame->mBowVec) {
          bowVectorKeys.push_back(pair.first);
          bowVectorValues.push_back(pair.second);
        }
        keyFrameBowVectorMsg.bow_vector_keys = bowVectorKeys;
        keyFrameBowVectorMsg.bow_vector_values = bowVectorValues;

        keyFrameBowVectorMsgs.push_back(keyFrameBowVectorMsg);
      }
    }

    cout << endl;

    // Send new keyframes message to agent
    interfaces::msg::NewKeyFrames msg;
    msg.key_frame_bow_vectors = keyFrameBowVectorMsgs;
    msg.sender_agent_id = agentId;
    newKeyFramesPubs[connectedAgentId]->publish(msg);

    // Add to sent key frames map
    sentKeyFrameUuids[connectedAgentId].insert(newKeyFrameUuids.begin(), newKeyFrameUuids.end());
  }
}

void OrbSlam3Wrapper::receiveNewKeyFrames(const interfaces::msg::NewKeyFrames::SharedPtr msg) {
  unique_lock<mutex> lock(mutexWrapper);

  vector<interfaces::msg::KeyFrameBowVector> newKeyFramesBowVectors = msg->key_frame_bow_vectors;
  cout << "received new key frame bow vectors";

  for (interfaces::msg::KeyFrameBowVector newKeyFramesBowVector : newKeyFramesBowVectors) {
    boost::uuids::uuid uuid = arrayToUuid(newKeyFramesBowVector.uuid); // Uuid of keyframe we want to get the map of

    vector<int64_t> bowVectorKeys = newKeyFramesBowVector.bow_vector_keys;
    vector<double> bowVectorValues = newKeyFramesBowVector.bow_vector_values;
    DBoW2::BowVector bowVector;

    for (size_t i = 0; i < bowVectorKeys.size(); i++) {
      bowVector.addWeight(bowVectorKeys[i], bowVectorValues[i]);
    }

    bool mergePossible = pSLAM->DetectMergePossibility(bowVector, uuid);

    if (mergePossible) {
      cout << "Merge possible!" << endl;
      auto request = std::make_shared<interfaces::srv::RequestMap::Request>();

      request->target_key_frame_uuid = newKeyFramesBowVector.uuid;

      vector<interfaces::msg::Uuid> dontIncludeUuids;
      for (ORB_SLAM3::KeyFrame* keyFrame : pSLAM->GetAllKeyFrames()) {
        interfaces::msg::Uuid uuidMsg;
        uuidMsg.uuid = uuidToArray(keyFrame->uuid);
        dontIncludeUuids.push_back(uuidMsg);
      }
      request->dont_include_uuids = dontIncludeUuids;

      auto future_result = requestMapClients[msg->sender_agent_id]->async_send_request(
        request, bind(&OrbSlam3Wrapper::handleRequestMapResponse, this, placeholders::_1));

      break;
    }
  }
}

void OrbSlam3Wrapper::handleRequestMapResponse(rclcpp::Client<interfaces::srv::RequestMap>::SharedFuture future) {
  unique_lock<mutex> lock(mutexWrapper);

  interfaces::srv::RequestMap::Response::SharedPtr response = future.get();

  RCLCPP_INFO(this->get_logger(), "Handling request map response. Received serialized map. Size: %d",
    response->serialized_map.size());

  pSLAM->AddSerializedMap(response->serialized_map);
}

boost::uuids::uuid OrbSlam3Wrapper::arrayToUuid(array<unsigned char, 16> array) {
  boost::uuids::uuid uuid;
  std::copy(array.begin(), array.end(), uuid.data);
  return uuid;
}

array<unsigned char, 16> OrbSlam3Wrapper::uuidToArray(boost::uuids::uuid uuid) {
  std::array<unsigned char, 16> array;
  std::copy(std::begin(uuid.data), std::end(uuid.data), array.begin());
  return array;
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