#include "orb_slam3_wrapper.h"

#include "DBoW2/DBoW2/BowVector.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Optimizer.h"
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cstddef>
#include <mutex>
#include <shared_mutex>
#include <vector>

#define MIN_MAP_SHARE_SIZE 5

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

  // Create subscriptions
  newKeyFrameBowsSub = this->create_subscription<interfaces::msg::NewKeyFrameBows>(node_name + "/new_key_frame_bows", 1,
    std::bind(&OrbSlam3Wrapper::receiveNewKeyFrameBows, this, std::placeholders::_1));
  successfullyMergedSub
    = this->create_subscription<interfaces::msg::SuccessfullyMerged>(node_name + "/successfully_merged", 1,
      std::bind(&OrbSlam3Wrapper::receiveSuccessfullyMergedMsg, this, std::placeholders::_1));
  newKeyFramesSub = this->create_subscription<interfaces::msg::NewKeyFrames>(
    node_name + "/new_key_frames", 1, std::bind(&OrbSlam3Wrapper::receiveNewKeyFrames, this, std::placeholders::_1));

  if (agentId == 1)
    connectedAgentIds = { 2 };
  else
    connectedAgentIds = { 1 };

  // TODO: create a proper topic handler that handles nodes connecting/disconnecting
  for (uint connectedAgentId : connectedAgentIds) {
    newKeyFrameBowsPubs[connectedAgentId] = this->create_publisher<interfaces::msg::NewKeyFrameBows>(
      "robot" + to_string(connectedAgentId) + "/new_key_frame_bows", 1);
    getCurrentMapClients[connectedAgentId]
      = this->create_client<interfaces::srv::GetCurrentMap>("robot" + to_string(connectedAgentId) + "/get_current_map");
    successfullyMergedPubs[connectedAgentId] = this->create_publisher<interfaces::msg::SuccessfullyMerged>(
      "robot" + to_string(connectedAgentId) + "/successfully_merged", 1);
    newKeyFramesPubs[connectedAgentId] = this->create_publisher<interfaces::msg::NewKeyFrames>(
      "robot" + to_string(connectedAgentId) + "/new_key_frames", 1);

    successfullyMerged[connectedAgentId] = false;
  }

  shareNewKeyFrameBowsTimer = this->create_wall_timer(2s, std::bind(&OrbSlam3Wrapper::sendNewKeyFrameBows, this));
  shareNewKeyFramesTimer = this->create_wall_timer(2s, std::bind(&OrbSlam3Wrapper::sendNewKeyFrames, this));
};

void OrbSlam3Wrapper::handleGetCurrentMapRequest(const std::shared_ptr<interfaces::srv::GetCurrentMap::Request> request,
  std::shared_ptr<interfaces::srv::GetCurrentMap::Response> response) {
  unique_lock<mutex> lock(mutexWrapper);

  cout << "Handling get current map request" << endl;

  // Get the lastest keyframe as the reference keyframe uuid
  ORB_SLAM3::KeyFrame* latestKeyFrame = nullptr;
  for (ORB_SLAM3::KeyFrame* keyFrame : pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames()) {
    if (!latestKeyFrame || (keyFrame->mnId > latestKeyFrame->mnId && keyFrame->creatorAgentId == agentId)) {
      latestKeyFrame = keyFrame;
    }
  }
  referenceKeyFrame[request->sender_agent_id] = latestKeyFrame;

  // Clone current map
  ORB_SLAM3::Map* currentMapCopy = deepCopyMap(pSLAM->GetAtlas()->GetCurrentMap());

  // Remove keyframes not from this agent
  for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
    if (keyFrame->creatorAgentId != agentId) {
      keyFrame->SetBadFlag(true);
    }
  }

  response->next_reference_key_frame_uuid = uuidToArray(latestKeyFrame->uuid);
  response->sender_agent_id = agentId;
  response->serialized_map = pSLAM->SerializeMap(currentMapCopy);
}

void OrbSlam3Wrapper::handleGetCurrentMapResponse(rclcpp::Client<interfaces::srv::GetCurrentMap>::SharedFuture future) {
  unique_lock<mutex> lock(mutexWrapper);

  interfaces::srv::GetCurrentMap::Response::SharedPtr response = future.get();

  RCLCPP_INFO(this->get_logger(), "Handling get current map response. Received serialized map. Size: %d",
    response->serialized_map.size());

  pSLAM->AddSerializedMap(response->serialized_map);

  // Set the next reference keyframe to not erase so it can be used as a reference
  // TODO: actually set not erase for this kf
  pSLAM->GetKeyFrameDatabase()
    ->ConvertUuidToKeyFrame(arrayToUuid(response->next_reference_key_frame_uuid))
    ->SetNotErase();

  // TODO: send the successfully merged message when it is actually merged
  interfaces::msg::SuccessfullyMerged msg;
  msg.successfully_merged = true;
  msg.sender_agent_id = agentId;
  successfullyMergedPubs[response->sender_agent_id]->publish(msg);
}

void OrbSlam3Wrapper::handleAddMapRequest(const std::shared_ptr<interfaces::srv::AddMap::Request> request,
  std::shared_ptr<interfaces::srv::AddMap::Response> response) {
  unique_lock<mutex> lock(mutexWrapper);

  RCLCPP_INFO(this->get_logger(), "Received serialized map. Size: %d", request->serialized_map.size());

  pSLAM->AddSerializedMap(request->serialized_map);
}

void OrbSlam3Wrapper::sendNewKeyFrames() {
  unique_lock<mutex> lock(mutexWrapper);

  // Send new key frames to all peers
  for (uint connectedAgentId : connectedAgentIds) {
    if (!successfullyMerged[connectedAgentId]) {
      continue;
    }

    ORB_SLAM3::Map* currentMapCopy = deepCopyMap(pSLAM->GetAtlas()->GetCurrentMap());

    // Remove keyframes not from this agent or have already been sent
    // keep all keyframe connections to reconnect later
    for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
      if (keyFrame->creatorAgentId != agentId || sentKeyFrameUuids[connectedAgentId].count(keyFrame->uuid) != 0) {
        currentMapCopy->EraseKeyFrame(keyFrame);
      }
    }

    // Remove mappoints that dont have one of the remaning kfs as their reference kf
    vector<ORB_SLAM3::KeyFrame*> remainingKeyFramesVec = currentMapCopy->GetAllKeyFrames();
    set<ORB_SLAM3::KeyFrame*> remainingKeyFramesSet = set(remainingKeyFramesVec.begin(), remainingKeyFramesVec.end());
    for (ORB_SLAM3::MapPoint* mapPoint : currentMapCopy->GetAllMapPoints()) {
      if (!mapPoint->GetReferenceKeyFrame() || remainingKeyFramesSet.count(mapPoint->GetReferenceKeyFrame()) == 0) {
        mapPoint->SetBadFlag();
      }
    }

    if (currentMapCopy->GetAllKeyFrames().size() < MIN_MAP_SHARE_SIZE) {
      continue;
    }

    // Transform keyframes and map points to move the reference keyframe to the origin
    Sophus::SE3f referenceKeyFramePoseInv = referenceKeyFrame[connectedAgentId]->GetPoseInverse();

    for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
      Sophus::SE3f newPose = keyFrame->GetPose() * referenceKeyFramePoseInv;
      keyFrame->SetPose(newPose);
    }
    for (ORB_SLAM3::MapPoint* mapPoint : currentMapCopy->GetAllMapPoints()) {
      Eigen::Vector3f newWorldPos = referenceKeyFramePoseInv.translation() + mapPoint->GetWorldPos();
      mapPoint->SetWorldPos(newWorldPos);

      Eigen::Vector3f newNormal = referenceKeyFramePoseInv.rotationMatrix() * mapPoint->GetNormal();
      mapPoint->SetNormalVector(newNormal);
    }

    // Add keyframes to sent keyframes map
    for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
      sentKeyFrameUuids[connectedAgentId].insert(keyFrame->uuid);
    }

    // Get the lastest keyframe to later make as the reference keyframe uuid
    ORB_SLAM3::KeyFrame* latestKeyFrame = nullptr;
    for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
      if (!latestKeyFrame || (keyFrame->mnId > latestKeyFrame->mnId && keyFrame->creatorAgentId == agentId)) {
        // Get KF from current map, because this keyframe has had its coordinate frame changed
        latestKeyFrame = pSLAM->GetKeyFrameDatabase()->ConvertUuidToKeyFrame(keyFrame->uuid);
      }
    }
    ORB_SLAM3::KeyFrame* nextReferenceKeyFrame = latestKeyFrame;

    // Send new frames to peer
    interfaces::msg::NewKeyFrames msg;
    msg.sender_agent_id = agentId;
    msg.serialized_map = pSLAM->SerializeMap(currentMapCopy);
    msg.reference_key_frame_uuid = uuidToArray(referenceKeyFrame[connectedAgentId]->uuid);
    msg.next_reference_key_frame_uuid = uuidToArray(nextReferenceKeyFrame->uuid);
    newKeyFramesPubs[connectedAgentId]->publish(msg);

    // Set reference KF as erase and next reference KF as no erase
    referenceKeyFrame[connectedAgentId]->SetErase();
    nextReferenceKeyFrame->SetNotErase();

    // Actually set referenceKeyFrameUuid
    referenceKeyFrame[connectedAgentId] = nextReferenceKeyFrame;

    cout << "Sent new key frames" << endl;
  }
}

void OrbSlam3Wrapper::receiveNewKeyFrames(const interfaces::msg::NewKeyFrames::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received new key frames. Size: %d", msg->serialized_map.size());

  ORB_SLAM3::Map* newMap = pSLAM->GetAtlas()->DeserializeMap(msg->serialized_map);

  // Transform keyframes and map points to move origin to the reference keyframe
  ORB_SLAM3::KeyFrame* referenceKeyFrame
    = pSLAM->GetKeyFrameDatabase()->ConvertUuidToKeyFrame(arrayToUuid(msg->reference_key_frame_uuid));
  Sophus::SE3f referenceKeyFramePose = referenceKeyFrame->GetPose();
  for (ORB_SLAM3::KeyFrame* keyFrame : newMap->GetAllKeyFrames()) {
    Sophus::SE3f newPose = keyFrame->GetPose() * referenceKeyFramePose;
    keyFrame->SetPose(newPose);
  }
  for (ORB_SLAM3::MapPoint* mapPoint : newMap->GetAllMapPoints()) {
    Eigen::Vector3f newWorldPos = referenceKeyFramePose.translation() + mapPoint->GetWorldPos();
    mapPoint->SetWorldPos(newWorldPos);

    Eigen::Vector3f newNormal = referenceKeyFramePose.rotationMatrix() * mapPoint->GetNormal();
    mapPoint->SetNormalVector(newNormal);
  }

  // Allow reference KF to be erased and stop next reference KF from being erased
  referenceKeyFrame->SetErase();
  pSLAM->GetKeyFrameDatabase()->ConvertUuidToKeyFrame(arrayToUuid(msg->next_reference_key_frame_uuid))->SetNotErase();

  ORB_SLAM3::Map* currentMap = pSLAM->GetAtlas()->GetCurrentMap();

  // Move keyframes to current map
  for (ORB_SLAM3::KeyFrame* keyFrame : newMap->GetAllKeyFrames()) {
    currentMap->AddKeyFrame(keyFrame);
    keyFrame->UpdateMap(currentMap);
  }

  // Move map points to current map, while also deduplicating
  for (ORB_SLAM3::MapPoint* mapPoint : newMap->GetAllMapPoints()) {
    if (mapPoint->isBad()) {
      continue;
    }

    currentMap->AddMapPoint(mapPoint);
    mapPoint->UpdateMap(currentMap);
  }

  // Update links in the Covisibility Graph
  for (ORB_SLAM3::KeyFrame* keyFrame : currentMap->GetAllKeyFrames()) {
    // keyFrame->UpdateConnections();
  }
}

void OrbSlam3Wrapper::sendNewKeyFrameBows() {
  unique_lock<mutex> lock(mutexWrapper);

  vector<ORB_SLAM3::KeyFrame*> keyFrames = pSLAM->GetAllKeyFrames();

  // Send new key frame bows to all peers
  for (uint connectedAgentId : connectedAgentIds) {
    if (successfullyMerged[connectedAgentId]) {
      continue;
    }

    set<boost::uuids::uuid> newKeyFrameBowUuids;
    vector<interfaces::msg::KeyFrameBowVector> keyFrameBowVectorMsgs;

    for (ORB_SLAM3::KeyFrame* keyFrame : keyFrames) {
      if (keyFrame->creatorAgentId == agentId && sentKeyFrameBowUuids[connectedAgentId].count(keyFrame->uuid) == 0) {
        interfaces::msg::KeyFrameBowVector keyFrameBowVectorMsg;

        // Set uuid
        std::array<unsigned char, 16> uuidArray = uuidToArray(keyFrame->uuid);
        keyFrameBowVectorMsg.uuid = uuidArray;
        newKeyFrameBowUuids.insert(keyFrame->uuid);

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

    if (keyFrameBowVectorMsgs.size() > MIN_MAP_SHARE_SIZE) {
      // Send new keyframes message to agent
      cout << "sent new key frame bows" << endl;
      interfaces::msg::NewKeyFrameBows msg;
      msg.key_frame_bow_vectors = keyFrameBowVectorMsgs;
      msg.sender_agent_id = agentId;
      newKeyFrameBowsPubs[connectedAgentId]->publish(msg);

      // Add to sent key frames map
      sentKeyFrameBowUuids[connectedAgentId].insert(newKeyFrameBowUuids.begin(), newKeyFrameBowUuids.end());
    }
  }
}

void OrbSlam3Wrapper::receiveNewKeyFrameBows(const interfaces::msg::NewKeyFrameBows::SharedPtr msg) {
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
      auto request = std::make_shared<interfaces::srv::GetCurrentMap::Request>();
      request->sender_agent_id = agentId;
      auto future_result = getCurrentMapClients[msg->sender_agent_id]->async_send_request(
        request, bind(&OrbSlam3Wrapper::handleGetCurrentMapResponse, this, placeholders::_1));

      break;
    }
  }
}

void OrbSlam3Wrapper::receiveSuccessfullyMergedMsg(const interfaces::msg::SuccessfullyMerged::SharedPtr msg) {
  successfullyMerged[msg->sender_agent_id] = msg->successfully_merged;

  // Add keyframes to sent keyframes map
  for (ORB_SLAM3::KeyFrame* keyFrame : pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames()) {
    sentKeyFrameUuids[msg->sender_agent_id].insert(keyFrame->uuid);
  }
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

ORB_SLAM3::Map* OrbSlam3Wrapper::deepCopyMap(ORB_SLAM3::Map* targetMap) {
  // Make a deep copy of the target map
  // TODO: make this less terrible!
  ORB_SLAM3::Map* mapCopy;

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
  ia >> mapCopy;
  mapCopy->PostLoad(dummyKFDB, dummyORBVoc, dummyMCams);

  return mapCopy;
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