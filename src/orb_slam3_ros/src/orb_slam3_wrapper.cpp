#include "orb_slam3_wrapper.h"

#include "DBoW2/DBoW2/BowVector.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Optimizer.h"
#include "System.h"
#include "peer.h"
#include "sophus/se3.hpp"
#include "sophus/sim3.hpp"
#include "sophus/so3.hpp"
#include "sophus/types.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cstddef>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <interfaces/msg/map_point.hpp>
#include <interfaces/msg/sim3_transform_stamped.hpp>
#include <interfaces/msg/uuid.hpp>
#include <interfaces/srv/get_map_points.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <shared_mutex>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

#define MIN_KEY_FRAME_SHARE_SIZE 5
#define MIN_BOW_SHARE_SIZE 12
#define MIN_MAP_POINTS_FOR_SCALE_ADJUSTMENT 500

using namespace std;

OrbSlam3Wrapper::OrbSlam3Wrapper(string node_name, string voc_file, ORB_SLAM3::System::eSensor sensor_type)
  : Node("robot" + to_string(agentId))
  , node_handle_(std::shared_ptr<OrbSlam3Wrapper>(this, [](auto*) {}))
  , image_transport(node_handle_) {
  // Create parameters
  this->declare_parameter("agentId", 1);
  agentId = this->get_parameter("agentId").as_int();

  this->declare_parameter("config",
    "/home/joshuabird/Desktop/Parallels\ Shared\ Folders/"
    "ubuntuSharedFolder/part_II_project/src/webots_sim/"
    "worlds/webots.yaml");
  settings_file = this->get_parameter("config").as_string();

  node_name = "robot" + to_string(agentId);
  RCLCPP_INFO(this->get_logger(), node_name.c_str());

  Eigen::Matrix3d rotation_matrix;
  rotation_matrix << 1, 0, 0, 0, 0, 1, 0, -1, 0;
  Eigen::Quaterniond quaternion(rotation_matrix);
  Eigen::Vector3d translation(0, 0, 0);
  world_to_origin = Sophus::Sim3d(quaternion, translation);
  world_to_origin.setScale(1);
  origin_frame_id = node_name + "/origin";
  slam_system_frame_id = origin_frame_id;

  this->sensor_type = sensor_type;
  pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, agentId, true);

  baseMap = pSLAM->GetAtlas()->GetCurrentMap();

  // Create publishers
  pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>(node_name + "/camera_pose", 1);
  pose_marker_pub = this->create_publisher<visualization_msgs::msg::Marker>(node_name + "/camera_pose_marker", 1);
  tracked_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/tracked_points", 1);
  all_mappoints_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>(node_name + "/all_points", 1);
  tracking_img_pub = image_transport.advertise(node_name + "/tracking_image", 1);
  kf_markers_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(node_name + "/kf_markers", 1000);
  if (sensor_type == ORB_SLAM3::System::IMU_MONOCULAR || sensor_type == ORB_SLAM3::System::IMU_STEREO
    || sensor_type == ORB_SLAM3::System::IMU_RGBD) {
    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(node_name + "/body_odom", 1);
  }
  sim3_transform_pub = this->create_publisher<interfaces::msg::Sim3TransformStamped>("/sim3_transform", 1);

  // Create services
  get_current_map_service = this->create_service<interfaces::srv::GetCurrentMap>(node_name + "/get_current_map",
    std::bind(&OrbSlam3Wrapper::handleGetCurrentMapRequest, this, std::placeholders::_1, std::placeholders::_2));
  getMapPointsService = this->create_service<interfaces::srv::GetMapPoints>(node_name + "/get_map_points",
    std::bind(&OrbSlam3Wrapper::handleGetMapPointsRequest, this, std::placeholders::_1, std::placeholders::_2));

  // Create subscriptions
  newKeyFrameBowsSub = this->create_subscription<interfaces::msg::NewKeyFrameBows>(node_name + "/new_key_frame_bows", 1,
    std::bind(&OrbSlam3Wrapper::receiveNewKeyFrameBows, this, std::placeholders::_1));
  successfullyMergedSub
    = this->create_subscription<interfaces::msg::SuccessfullyMerged>(node_name + "/successfully_merged", 1,
      std::bind(&OrbSlam3Wrapper::receiveSuccessfullyMergedMsg, this, std::placeholders::_1));
  newKeyFramesSub = this->create_subscription<interfaces::msg::NewKeyFrames>(
    node_name + "/new_key_frames", 1, std::bind(&OrbSlam3Wrapper::receiveNewKeyFrames, this, std::placeholders::_1));
  isLostFromBaseMapSub
    = this->create_subscription<interfaces::msg::IsLostFromBaseMap>(node_name + "/is_lost_from_base_map", 1,
      std::bind(&OrbSlam3Wrapper::receiveIsLostFromBaseMapMsg, this, std::placeholders::_1));
  loopClosureTriggersSub
    = this->create_subscription<interfaces::msg::LoopClosureTriggers>(node_name + "/loop_closure_triggers", 1,
      std::bind(&OrbSlam3Wrapper::receiveLoopClosureTriggers, this, std::placeholders::_1));

  // TODO: create a proper topic handler that handles nodes connecting/disconnecting
  vector<uint> connectedPeerIds;
  if (agentId == 1)
    connectedPeerIds = { 2 };
  else
    connectedPeerIds = { 1 };

  for (uint connectedPeerId : connectedPeerIds) {
    connectedPeers[connectedPeerId] = new Peer(this->shared_from_this(), connectedPeerId);
  }

  updateScaleTimer = this->create_wall_timer(5s, std::bind(&OrbSlam3Wrapper::updateMapScale, this));

  resetVisualization();

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
};

void OrbSlam3Wrapper::run() {
  while (true) {
    if (newFrameProcessed) {
      updateSuccessfullyMerged();
      updateIsLostFromBaseMap();
      sendNewKeyFrameBows();
      sendNewKeyFrames();
      publish_topics(lastFrameTimestamp);

      newFrameProcessed = false;
    }

    rclcpp::spin_some(this->node_handle_);
  }
}

void OrbSlam3Wrapper::handleGetCurrentMapRequest(const std::shared_ptr<interfaces::srv::GetCurrentMap::Request> request,
  std::shared_ptr<interfaces::srv::GetCurrentMap::Response> response) {
  unique_lock<mutex> lock(mutexWrapper);
  unique_lock<mutex> lock2(pSLAM->GetAtlas()->GetCurrentMap()->mMutexMapUpdate); // Wait for map merge

  cout << "Handling get current map request" << endl;

  // Get the lastest keyframe as the reference keyframe uuid
  ORB_SLAM3::KeyFrame* latestKeyFrame = nullptr;
  for (ORB_SLAM3::KeyFrame* keyFrame : pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames()) {
    if (!latestKeyFrame || (keyFrame->mnId > latestKeyFrame->mnId && keyFrame->creatorAgentId == agentId)) {
      latestKeyFrame = keyFrame;
    }
  }
  connectedPeers[request->sender_agent_id]->setReferenceKeyFrame(latestKeyFrame);

  // Clone current map
  unique_ptr<ORB_SLAM3::Map> currentMapCopy = deepCopyMap(pSLAM->GetAtlas()->GetCurrentMap());

  // Remove keyframes not from this agent
  for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
    if (keyFrame->creatorAgentId != agentId) {
      keyFrame->SetBadFlag(true);
    }
  }

  response->sender_agent_id = agentId;
  response->serialized_map = pSLAM->SerializeMap(currentMapCopy.get());
  response->merge_candidate_key_frame_uuids = request->merge_candidate_key_frame_uuids;
}

void OrbSlam3Wrapper::handleGetCurrentMapResponse(rclcpp::Client<interfaces::srv::GetCurrentMap>::SharedFuture future) {
  unique_lock<mutex> lock(mutexWrapper);

  interfaces::srv::GetCurrentMap::Response::SharedPtr response = future.get();

  RCLCPP_INFO(this->get_logger(), "Handling get current map response. Received serialized map. Size: %d",
    response->serialized_map.size());

  // Get mergeCandidateKeyFrameUuids, which was generated by us when we created the getCurrentMap request
  vector<boost::uuids::uuid> mergeCandidateKeyFrameUuids;
  for (interfaces::msg::Uuid uuidMsg : response->merge_candidate_key_frame_uuids) {
    mergeCandidateKeyFrameUuids.push_back(arrayToUuid(uuidMsg.uuid));
  }

  pSLAM->AddSerializedMapToTryMerge(response->serialized_map, mergeCandidateKeyFrameUuids);

  // Set the next reference keyframe to not erase so it can be used as a reference
  // TODO: actually set not erase for this kf
  pSLAM->GetKeyFrameDatabase()
    ->ConvertUuidToKeyFrame(arrayToUuid(response->next_reference_key_frame_uuid))
    ->SetNotErase();
}

void OrbSlam3Wrapper::sendNewKeyFrames() {
  unique_lock<mutex> lock(mutexWrapper);

  if (isLostFromBaseMap)
    return;

  // Send new key frames to all peers
  for (auto& pair : connectedPeers) {
    Peer* connectedPeer = pair.second;

    if (!connectedPeer->getRemoteSuccessfullyMerged() || connectedPeer->getIsLostFromBaseMap()) {
      continue;
    }

    // Check if we have enough keyframes to send update
    int newKeyFrames = 0;
    for (ORB_SLAM3::KeyFrame* keyFrame : pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames()) {
      if (connectedPeer->getSentKeyFrameUuids().count(keyFrame->uuid) == 0 && keyFrame->creatorAgentId == agentId
        && !keyFrame->isBad())
        newKeyFrames++;
    }
    if (newKeyFrames < MIN_KEY_FRAME_SHARE_SIZE)
      continue;

    unique_ptr<ORB_SLAM3::Map> currentMapCopy = deepCopyMap(pSLAM->GetAtlas()->GetCurrentMap());

    // Remove keyframes not from this agent or have already been sent
    // keep all keyframe connections to reconnect later
    vector<ORB_SLAM3::KeyFrame*> keyFramesToDelete;
    int a = currentMapCopy->GetMaxKFid();
    for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
      if (keyFrame->creatorAgentId != agentId
        || connectedPeer->getSentKeyFrameUuids().count(keyFrame->uuid) != 0
        // only send keyframes once they are outside of the mappoint culling window
        || keyFrame->isBad()) {
        currentMapCopy->EraseKeyFrame(keyFrame);
        keyFramesToDelete.push_back(keyFrame);
      }
    }

    // Remove mappoints that dont have one of the remaning kfs as their reference kf
    vector<ORB_SLAM3::KeyFrame*> remainingKeyFramesVec = currentMapCopy->GetAllKeyFrames();
    set<ORB_SLAM3::KeyFrame*> remainingKeyFramesSet = set(remainingKeyFramesVec.begin(), remainingKeyFramesVec.end());
    for (ORB_SLAM3::MapPoint* mapPoint : currentMapCopy->GetAllMapPoints()) {
      if (!mapPoint->GetReferenceKeyFrame() || remainingKeyFramesSet.count(mapPoint->GetReferenceKeyFrame()) == 0
        || mapPoint->isBad()) {
        mapPoint->SetBadFlag();
        delete mapPoint;
      }
    }

    // Transform keyframes and map points to move the reference keyframe to the origin
    Sophus::SE3f referenceKeyFramePoseInv = connectedPeer->getReferenceKeyFrame()->GetPoseInverse();

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
      connectedPeer->addSentKeyFrameUuid(keyFrame->uuid);
    }

    // Add map points to sent map points map
    for (ORB_SLAM3::MapPoint* mapPoint : currentMapCopy->GetAllMapPoints()) {
      connectedPeer->addSentMapPointUuid(mapPoint->uuid);
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
    msg.header.stamp = lastFrameTimestamp;
    msg.sender_agent_id = agentId;
    msg.serialized_map = pSLAM->SerializeMap(currentMapCopy.get());
    msg.reference_key_frame_uuid = uuidToArray(connectedPeer->getReferenceKeyFrame()->uuid);
    msg.next_reference_key_frame_uuid = uuidToArray(nextReferenceKeyFrame->uuid);
    connectedPeer->newKeyFramesPub->publish(msg);

    // Set reference KF as erase and next reference KF as no erase
    connectedPeer->getReferenceKeyFrame()->SetErase();
    nextReferenceKeyFrame->SetNotErase();

    // Actually set referenceKeyFrameUuid
    connectedPeer->setReferenceKeyFrame(nextReferenceKeyFrame);

    for (ORB_SLAM3::KeyFrame* keyFrame : keyFramesToDelete) { // TODO: REALLY REALLY should move to smart pointers
      delete keyFrame;
    }

    RCLCPP_INFO(this->get_logger(), "Sent new key frames");
  }
}

void OrbSlam3Wrapper::receiveNewKeyFrames(const interfaces::msg::NewKeyFrames::SharedPtr msg) {
  unique_lock<mutex> lock(mutexWrapper);

  RCLCPP_INFO(this->get_logger(), "Received new key frames. Size: %d", msg->serialized_map.size());

  ORB_SLAM3::Map* newMap = pSLAM->GetAtlas()->DeserializeMap(msg->serialized_map, true);
  ORB_SLAM3::KeyFrame* referenceKeyFrame
    = pSLAM->GetKeyFrameDatabase()->ConvertUuidToKeyFrame(arrayToUuid(msg->reference_key_frame_uuid));

  {
    unique_lock<mutex> lock2(pSLAM->GetAtlas()->GetCurrentMap()->mMutexMapUpdate);

    // Transform keyframes and map points to move origin to the reference keyframe
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
  }

  // Allow reference KF to be erased and stop next reference KF from being erased
  referenceKeyFrame->SetErase();
  pSLAM->GetKeyFrameDatabase()->ConvertUuidToKeyFrame(arrayToUuid(msg->next_reference_key_frame_uuid))->SetNotErase();

  ORB_SLAM3::Map* currentMap = pSLAM->GetAtlas()->GetCurrentMap();

  // Move map points to current map
  for (ORB_SLAM3::MapPoint* mapPoint : newMap->GetAllMapPoints()) {
    if (mapPoint->isBad()) {
      continue;
    }

    newMap->EraseMapPoint(mapPoint);
    // currentMap->AddMapPoint(mapPoint);
    mapPoint->UpdateMap(currentMap);
  }

  // Move keyframes to current map
  for (ORB_SLAM3::KeyFrame* keyFrame : newMap->GetAllKeyFrames()) {
    newMap->EraseKeyFrame(keyFrame);
    keyFrame->UpdateMap(currentMap);

    pSLAM->GetLocalMapper()->InsertExternalKeyFrame(keyFrame);
    // // Run bundle adjustment for new KFs and a local window around them
    // bool mbAbortBA = false; // no idea what this is used for
    // int num_FixedKF_BA = 0;
    // int num_OptKF_BA = 0;
    // int num_MPs_BA = 0;
    // int num_edges_BA = 0;
    // ORB_SLAM3::Optimizer::LocalBundleAdjustment(
    //   keyFrame, &mbAbortBA, currentMap, num_FixedKF_BA, num_OptKF_BA, num_MPs_BA, num_edges_BA);
  }

  // Update links in the Covisibility Graph
  for (ORB_SLAM3::KeyFrame* keyFrame : currentMap->GetAllKeyFrames()) {
    // keyFrame->UpdateConnections();
  }

  delete newMap;
}

void OrbSlam3Wrapper::sendNewKeyFrameBows() {
  unique_lock<mutex> lock(mutexWrapper);

  if (isLostFromBaseMap)
    return;

  vector<ORB_SLAM3::KeyFrame*> keyFrames = pSLAM->GetAllKeyFrames();

  if (keyFrames.size() < MIN_BOW_SHARE_SIZE)
    return;

  // Send new key frame bows to all peers
  for (auto& pair : connectedPeers) {
    Peer* connectedPeer = pair.second;

    if (connectedPeer->getRemoteSuccessfullyMerged() || connectedPeer->getIsLostFromBaseMap()) {
      continue;
    }

    set<boost::uuids::uuid> newKeyFrameBowUuids;
    vector<interfaces::msg::KeyFrameBowVector> keyFrameBowVectorMsgs;

    for (ORB_SLAM3::KeyFrame* keyFrame : keyFrames) {
      if (keyFrame->creatorAgentId == agentId && connectedPeer->getSentKeyFrameBowUuids().count(keyFrame->uuid) == 0) {
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

    // Send new keyframes message to agent
    RCLCPP_INFO(this->get_logger(), "sent new key frame bows");
    interfaces::msg::NewKeyFrameBows msg;
    msg.header.stamp = lastFrameTimestamp;
    msg.key_frame_bow_vectors = keyFrameBowVectorMsgs;
    msg.sender_agent_id = agentId;
    connectedPeer->newKeyFrameBowsPub->publish(msg);

    // Add to sent key frames map
    connectedPeer->addSentKeyFrameBowUuids(newKeyFrameBowUuids.begin(), newKeyFrameBowUuids.end());
  }
}

void OrbSlam3Wrapper::receiveNewKeyFrameBows(const interfaces::msg::NewKeyFrameBows::SharedPtr msg) {
  unique_lock<mutex> lock(mutexWrapper);

  vector<interfaces::msg::KeyFrameBowVector> newKeyFramesBowVectors = msg->key_frame_bow_vectors;
  RCLCPP_INFO(this->get_logger(), "received new key frame bow vectors");

  vector<boost::uuids::uuid> mergeCandidateKeyFrameUuids;

  for (interfaces::msg::KeyFrameBowVector newKeyFramesBowVector : newKeyFramesBowVectors) {
    boost::uuids::uuid uuid = arrayToUuid(newKeyFramesBowVector.uuid); // Uuid of keyframe we want to get the map of

    vector<int64_t> bowVectorKeys = newKeyFramesBowVector.bow_vector_keys;
    vector<double> bowVectorValues = newKeyFramesBowVector.bow_vector_values;
    DBoW2::BowVector bowVector;

    for (size_t i = 0; i < bowVectorKeys.size(); i++) {
      bowVector.addWeight(bowVectorKeys[i], bowVectorValues[i]);
    }

    auto [mergePossible, bestMatchKeyFrameUuid] = pSLAM->DetectMergePossibility(bowVector, uuid);

    if (mergePossible) {
      mergeCandidateKeyFrameUuids.push_back(uuid);
      mergeCandidateKeyFrameUuids.push_back(bestMatchKeyFrameUuid);
    }
  }

  if (mergeCandidateKeyFrameUuids.size() > 0) {
    RCLCPP_INFO(this->get_logger(), "Merge with peer possible!");

    vector<interfaces::msg::Uuid> mergeCandidateKeyFrameUuidMsgs;
    for (boost::uuids::uuid uuid : mergeCandidateKeyFrameUuids) {
      interfaces::msg::Uuid uuidMsg;
      uuidMsg.uuid = uuidToArray(uuid);
      mergeCandidateKeyFrameUuidMsgs.push_back(uuidMsg);
    }

    auto request = std::make_shared<interfaces::srv::GetCurrentMap::Request>();
    request->sender_agent_id = agentId;
    request->merge_candidate_key_frame_uuids
      = mergeCandidateKeyFrameUuidMsgs; // Keep this state with the request for us to use later
    auto future_result = connectedPeers[msg->sender_agent_id]->getCurrentMapClient->async_send_request(
      request, bind(&OrbSlam3Wrapper::handleGetCurrentMapResponse, this, placeholders::_1));
  }
}

void OrbSlam3Wrapper::updateSuccessfullyMerged() {
  unique_lock<mutex> lock(mutexWrapper);

  map<uint, pair<vector<boost::uuids::uuid>, Sophus::Sim3d>> successfullyMergedAgentIds
    = pSLAM->GetAtlas()->GetSuccessfullyMergedAgentIds();

  for (auto& pair : connectedPeers) {
    Peer* connectedPeer = pair.second;

    bool successfullyMerged = false;
    for (auto pair : successfullyMergedAgentIds) {
      uint successfullyMergedAgentId = pair.first;
      if (successfullyMergedAgentId == connectedPeer->getId())
        successfullyMerged = true;
    }

    if (connectedPeer->getLocalSuccessfullyMerged() != successfullyMerged) {
      if (successfullyMerged) {
        baseMap = pSLAM->GetAtlas()->GetCurrentMap();
      } // TODO: handle else case. is there even an else case?

      vector<boost::uuids::uuid> mergedKeyFrameUuid = successfullyMergedAgentIds[connectedPeer->getId()].first;
      Sophus::Sim3d mergeWorldToCurrentWorld = successfullyMergedAgentIds[connectedPeer->getId()].second;

      connectedPeer->setLocalSuccessfullyMerged(successfullyMerged);

      // Redefine our reference frame if we are moving to the other agents reference frame
      if (connectedPeer->getId() < agentId) {
        origin_frame_parent_id = "robot" + to_string(connectedPeer->getId()) + "/origin";
        slam_system_frame_id = origin_frame_parent_id;

        // Redefine our old reference frame
        ORB_SLAM3::KeyFrame* originKeyFrame = nullptr;
        for (ORB_SLAM3::KeyFrame* keyFrame : pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames()) {
          if (keyFrame->creatorAgentId == agentId) {
            if (originKeyFrame == nullptr || keyFrame->mnId < originKeyFrame->mnId)
              originKeyFrame = keyFrame;
          }
        }
        world_to_origin = world_to_origin * mergeWorldToCurrentWorld;
      }

      RCLCPP_INFO(this->get_logger(),
        ("Successfully merged with peer " + to_string(connectedPeer->getId()) + " "
          + (successfullyMerged ? "true" : "false"))
          .c_str());

      // Tell this agent that we are successfully merged
      interfaces::msg::SuccessfullyMerged msg;
      msg.header.stamp = lastFrameTimestamp;
      msg.successfully_merged = successfullyMerged;
      msg.sender_agent_id = agentId;
      for (boost::uuids::uuid mergedKeyFrameUuid : successfullyMergedAgentIds[connectedPeer->getId()].first) {
        interfaces::msg::Uuid uuidMsg;
        uuidMsg.uuid = uuidToArray(mergedKeyFrameUuid);
        msg.merged_key_frame_uuids.push_back(uuidMsg);
      }
      // Pass along all_key_frames_in_map so that peer knows not to resend these
      for (ORB_SLAM3::KeyFrame* keyFrame : pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames()) {
        if (keyFrame->creatorAgentId == connectedPeer->getId()) {
          interfaces::msg::Uuid uuidMsg;
          uuidMsg.uuid = uuidToArray(keyFrame->uuid);
          msg.all_key_frames_in_map.push_back(uuidMsg);
        }
      }
      connectedPeer->successfullyMergedPub->publish(msg);
    }
  }
}

void OrbSlam3Wrapper::receiveSuccessfullyMergedMsg(const interfaces::msg::SuccessfullyMerged::SharedPtr msg) {
  unique_lock<mutex> lock(mutexWrapper);

  connectedPeers[msg->sender_agent_id]->setRemoteSuccessfullyMerged(msg->successfully_merged);

  RCLCPP_INFO(this->get_logger(), "Recieved successful merge message from peer, now requesting its map");

  // Add existing keyframes to sent keyframes map
  set<boost::uuids::uuid> sentKeyFrameUuids;
  for (interfaces::msg::Uuid uuidMsg : msg->all_key_frames_in_map)
    sentKeyFrameUuids.insert(arrayToUuid(uuidMsg.uuid));

  connectedPeers[msg->sender_agent_id]->addSentKeyFrameUuids(sentKeyFrameUuids.begin(), sentKeyFrameUuids.end());

  if (!connectedPeers[msg->sender_agent_id]->getLocalSuccessfullyMerged()) {
    // Request map from that peer
    auto request = std::make_shared<interfaces::srv::GetCurrentMap::Request>();
    request->sender_agent_id = agentId;
    request->merge_candidate_key_frame_uuids
      = msg->merged_key_frame_uuids; // Pass this state along with the request so we can use it later
    auto future_result = connectedPeers[msg->sender_agent_id]->getCurrentMapClient->async_send_request(
      request, bind(&OrbSlam3Wrapper::handleGetCurrentMapResponse, this, placeholders::_1));
  }
}

void OrbSlam3Wrapper::updateIsLostFromBaseMap() {
  unique_lock<mutex> lock(mutexWrapper);

  bool newIsLostFromBaseMap = baseMap != pSLAM->GetAtlas()->GetCurrentMap();

  if (isLostFromBaseMap != newIsLostFromBaseMap) {
    RCLCPP_INFO(this->get_logger(), "Update lost from base map: %d", newIsLostFromBaseMap);

    isLostFromBaseMap = newIsLostFromBaseMap;

    // Update peers on if we are lost or not
    interfaces::msg::IsLostFromBaseMap msg;
    msg.header.stamp = lastFrameTimestamp;
    msg.sender_agent_id = agentId;
    msg.is_lost_from_base_map = isLostFromBaseMap;

    for (auto& pair : connectedPeers) {
      Peer* connectedPeer = pair.second;
      connectedPeer->isLostFromBaseMapPub->publish(msg);
    }
  }
}

void OrbSlam3Wrapper::receiveIsLostFromBaseMapMsg(const interfaces::msg::IsLostFromBaseMap::SharedPtr msg) {
  unique_lock<mutex> lock(mutexWrapper);

  connectedPeers[msg->sender_agent_id]->setIsLostFromBaseMap(msg->is_lost_from_base_map);
}

void OrbSlam3Wrapper::updateMapScale() {
  auto handleGetMapPointsResponse = [this](rclcpp::Client<interfaces::srv::GetMapPoints>::SharedFuture future) {
    unique_lock<mutex> lock(mutexWrapper);
    interfaces::srv::GetMapPoints::Response::SharedPtr response = future.get();

    RCLCPP_INFO(this->get_logger(), "Received peer's map points");

    map<boost::uuids::uuid, Eigen::Vector3f> peerMapPointUuidToPos;
    for (interfaces::msg::MapPoint peerMapPointMsg : response->map_points) {
      boost::uuids::uuid uuid = arrayToUuid(peerMapPointMsg.uuid);
      Eigen::Vector3f position(peerMapPointMsg.position[0], peerMapPointMsg.position[1], peerMapPointMsg.position[2]);
      peerMapPointUuidToPos[uuid] = position;
    }

    vector<Eigen::Vector3f> sourcePoints;
    vector<Eigen::Vector3f> targetPoints;

    for (ORB_SLAM3::MapPoint* mapPoint : pSLAM->GetAtlas()->GetCurrentMap()->GetAllMapPoints()) {
      if (peerMapPointUuidToPos.count(mapPoint->uuid) != 0) {
        sourcePoints.push_back(mapPoint->GetWorldPos());
        targetPoints.push_back(peerMapPointUuidToPos[mapPoint->uuid]);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Point matches: %d", sourcePoints.size());

    if (sourcePoints.size() < MIN_MAP_POINTS_FOR_SCALE_ADJUSTMENT)
      return;

    auto [transformation, scale] = ransacPointSetAlignment(sourcePoints, targetPoints, 50, 500, 1e-5);

    pSLAM->GetAtlas()->GetCurrentMap()->ApplyScaledRotation(transformation, scale);

    RCLCPP_INFO(this->get_logger(), "Applied transformation");
    cout << "translation: " << transformation.translation() << " rotation: " << transformation.rotationMatrix()
         << " & scale: " << scale;
  };

  RCLCPP_INFO(this->get_logger(), "Attempting to update map scale...");

  uint lowestConnectedPeerAgentId = connectedPeers.begin()->first;
  Peer* lowestConnectedPeer = connectedPeers[lowestConnectedPeerAgentId];

  if (agentId < lowestConnectedPeerAgentId || lowestConnectedPeer->getRemoteSuccessfullyMerged() == false
    || lowestConnectedPeer->getLocalSuccessfullyMerged() == false || lowestConnectedPeer->getIsLostFromBaseMap()) {
    return;
  }

  auto request = std::make_shared<interfaces::srv::GetMapPoints::Request>();
  auto future_result = lowestConnectedPeer->getMapPointsClient->async_send_request(request, handleGetMapPointsResponse);
}

void OrbSlam3Wrapper::handleGetMapPointsRequest(const std::shared_ptr<interfaces::srv::GetMapPoints::Request> request,
  std::shared_ptr<interfaces::srv::GetMapPoints::Response> response) {
  unique_lock<mutex> lock(mutexWrapper);

  vector<ORB_SLAM3::MapPoint*> mapPoints = pSLAM->GetAtlas()->GetCurrentMap()->GetAllMapPoints();

  for (ORB_SLAM3::MapPoint* mapPoint : mapPoints) {
    if (mapPoint->isBad())
      continue;

    interfaces::msg::MapPoint mapPointMsg;

    mapPointMsg.uuid = uuidToArray(mapPoint->uuid);
    array<float, 3> position;
    position[0] = mapPoint->GetWorldPos().x();
    position[1] = mapPoint->GetWorldPos().y();
    position[2] = mapPoint->GetWorldPos().z();
    mapPointMsg.position = position;

    response->map_points.push_back(mapPointMsg);
  }

  RCLCPP_INFO(this->get_logger(), "Sent map points");
}

void OrbSlam3Wrapper::sendLoopClosureTriggers() {
  unique_lock<mutex> lock(mutexWrapper);

  if (isLostFromBaseMap)
    return;

  set<boost::uuids::uuid> loopClosureTriggers = pSLAM->GetAtlas()->GetLoopClosureTriggers();

  for (auto& pair : connectedPeers) {
    Peer* connectedPeer = pair.second;

    // If the peer has not successfully merged, we dont need to send loop closure triggers
    if (!connectedPeer->getRemoteSuccessfullyMerged()) {
      connectedPeer->addSentLoopClosureTriggerUuids(loopClosureTriggers.begin(), loopClosureTriggers.end());
      continue;
    }

    if (connectedPeer->getIsLostFromBaseMap()) {
      continue;
    }

    vector<interfaces::msg::Uuid> loopClosureTriggerUuids;
    for (boost::uuids::uuid triggerKeyFrameUuid : loopClosureTriggers) {
      if (connectedPeer->getSentLoopClosureTriggerUuids().count(triggerKeyFrameUuid) == 0) {
        connectedPeer->addSentLoopClosureTriggerUuid(triggerKeyFrameUuid);
        interfaces::msg::Uuid uuidMsg;
        uuidMsg.uuid = uuidToArray(triggerKeyFrameUuid);
        loopClosureTriggerUuids.push_back(uuidMsg);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Sent loop closure triggers");

    // Send loop closure triggers to peer
    interfaces::msg::LoopClosureTriggers msg;
    msg.header.stamp = lastFrameTimestamp;
    msg.sender_agent_id = agentId;
    msg.trigger_key_frame_uuids = loopClosureTriggerUuids;
    connectedPeer->loopClosureTriggersPub->publish(msg);
  }
}

void OrbSlam3Wrapper::receiveLoopClosureTriggers(const interfaces::msg::LoopClosureTriggers::SharedPtr msg) {
  unique_lock<mutex> lock(mutexWrapper);

  RCLCPP_INFO(this->get_logger(), "Received loop closure triggers");

  for (interfaces::msg::Uuid uuidMsg : msg->trigger_key_frame_uuids) {
    boost::uuids::uuid uuid = arrayToUuid(uuidMsg.uuid);
    ORB_SLAM3::KeyFrame* keyFrame = pSLAM->GetKeyFrameDatabase()->ConvertUuidToKeyFrame(uuid);
    pSLAM->GetLoopCloser()->InsertKeyFrame(keyFrame);
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

unique_ptr<ORB_SLAM3::Map> OrbSlam3Wrapper::deepCopyMap(ORB_SLAM3::Map* targetMap) {
  // Make a deep copy of the target map
  // TODO: make this less terrible!
  ORB_SLAM3::Map* mapCopy;

  // HACKY: create dummy objects for post load
  vector<ORB_SLAM3::GeometricCamera*> mvpCameras = pSLAM->GetAtlas()->GetAllCameras();
  set<ORB_SLAM3::GeometricCamera*> dummySCams(mvpCameras.begin(), mvpCameras.end());
  ORB_SLAM3::ORBVocabulary* dummyORBVoc = pSLAM->GetORBVocabulary();
  delete dummyKFDB;
  dummyKFDB = new ORB_SLAM3::KeyFrameDatabase(*dummyORBVoc);
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

  return unique_ptr<ORB_SLAM3::Map>(mapCopy);
}

void OrbSlam3Wrapper::publish_topics(rclcpp::Time msg_time, Eigen::Vector3f Wbb) {
  unique_lock<mutex> lock(mutexWrapper);

  Sophus::SE3f Twc = pSLAM->GetCamTwc();

  if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0)) // avoid publishing NaN
    return;

  // Publish origin transformation
  Sophus::SE3d world_to_origin_se3d(world_to_origin.quaternion(), world_to_origin.translation());
  publish_tf_transform(world_to_origin_se3d, origin_frame_parent_id, origin_frame_id, msg_time);
  publish_sim3_transform(world_to_origin, origin_frame_parent_id, origin_frame_id, msg_time);

  // Common topics
  // publish_tf_transform(Twc, origin_frame_id, cam_frame_id, msg_time);
  // publish_tracking_img(pSLAM->GetCurrentFrame(), msg_time);
  publish_tracked_points(pSLAM->GetTrackedMapPoints(), msg_time);
  publish_all_points(pSLAM->GetAllMapPoints(), msg_time);
  publish_keyframes(pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames(), msg_time);

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

    // publish_tf_transform(Twb, origin_frame_id, imu_frame_id, msg_time);
    publish_body_odom(Twb, Vwb, Wwb, msg_time);
  }
}

void OrbSlam3Wrapper::publish_camera_pose(Sophus::SE3f Tcw_SE3f, rclcpp::Time msg_time) {
  visualization_msgs::msg::Marker cameraPoseWireframe;
  cameraPoseWireframe.header.frame_id = slam_system_frame_id;
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
  poseStampedMsg.header.frame_id = slam_system_frame_id;
  poseStampedMsg.pose.position.x = Tcw_SE3f.translation().x();
  poseStampedMsg.pose.position.y = Tcw_SE3f.translation().y();
  poseStampedMsg.pose.position.z = Tcw_SE3f.translation().z();
  poseStampedMsg.pose.orientation.w = Tcw_SE3f.unit_quaternion().coeffs().w();
  poseStampedMsg.pose.orientation.x = Tcw_SE3f.unit_quaternion().coeffs().x();
  poseStampedMsg.pose.orientation.y = Tcw_SE3f.unit_quaternion().coeffs().y();
  poseStampedMsg.pose.orientation.z = Tcw_SE3f.unit_quaternion().coeffs().z();

  pose_pub->publish(poseStampedMsg);
}

void OrbSlam3Wrapper::publish_tf_transform(const Sophus::SE3d& T_SE3d, const std::string& frame_id,
  const std::string& child_frame_id, const rclcpp::Time& msg_time) {
  static tf2_ros::TransformBroadcaster tf_broadcaster(*this);

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

void OrbSlam3Wrapper::publish_sim3_transform(const Sophus::Sim3d& T_Sim3d, const std::string& frame_id,
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

void OrbSlam3Wrapper::publish_tracking_img(cv::Mat image, rclcpp::Time msg_time) {
  std_msgs::msg::Header header;
  header.stamp = msg_time;
  header.frame_id = cam_frame_id;

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

void OrbSlam3Wrapper::publish_keyframes(std::vector<ORB_SLAM3::KeyFrame*> keyFrames, rclcpp::Time msg_time) {
  visualization_msgs::msg::MarkerArray keyFrameMarkers;

  // Create keyframe wireframes
  for (ORB_SLAM3::KeyFrame* keyFrame : keyFrames) {
    if (keyFrame->isBad())
      continue;

    visualization_msgs::msg::Marker keyFrameWireframe;
    keyFrameWireframe.header.frame_id = slam_system_frame_id;
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
    connectedKeyFrameLines.header.frame_id = slam_system_frame_id;
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

// void OrbSlam3Wrapper::publish_kf_markers(std::vector<Sophus::SE3f> vKFposes, rclcpp::Time msg_time) {

void OrbSlam3Wrapper::publish_body_odom(
  Sophus::SE3f Twb_SE3f, Eigen::Vector3f Vwb_E3f, Eigen::Vector3f ang_vel_body, rclcpp::Time msg_time) {
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.child_frame_id = imu_frame_id;
  odom_msg.header.frame_id = origin_frame_id;
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

void OrbSlam3Wrapper::resetVisualization() {

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

sensor_msgs::msg::PointCloud2 OrbSlam3Wrapper::mappoint_to_pointcloud(
  std::vector<ORB_SLAM3::MapPoint*> map_points, rclcpp::Time msg_time) {
  const int num_channels = 3; // x y z

  if (map_points.size() == 0) {
    RCLCPP_INFO(this->get_logger(), "Map point vector is empty!");
  }

  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.stamp = msg_time;
  cloud.header.frame_id = slam_system_frame_id;
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

tuple<Sophus::SE3f, float> OrbSlam3Wrapper::ransacPointSetAlignment(vector<Eigen::Vector3f> sourcePoints,
  vector<Eigen::Vector3f> targetPoints, int numSamples, int iterations, float inlierThreshold) {
  vector<pair<Eigen::Vector3f, Eigen::Vector3f>> sourceTargetPointPairs;
  for (int i = 0; i < sourcePoints.size(); i++) {
    sourceTargetPointPairs.push_back(make_pair(sourcePoints[i], targetPoints[i]));
  }

  float targetVariance = 0;
  for (Eigen::Vector3f point : targetPoints) {
    targetVariance += point.squaredNorm();
  }
  targetVariance /= targetPoints.size();

  RCLCPP_INFO(this->get_logger(), "Target variance: %f", targetVariance);

  tuple<Sophus::SE3f, float> bestModel = pointSetAlignment(sourcePoints, targetPoints);
  int maxNumInliers = 0;

  for (int i = 0; i < iterations; i++) {
    // randomly select samples
    std::vector<pair<Eigen::Vector3f, Eigen::Vector3f>> samples = sourceTargetPointPairs;
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(samples.begin(), samples.end(), g);
    samples.resize(std::min(numSamples, static_cast<int>(samples.size())));

    vector<Eigen::Vector3f> sourceSamplePoints;
    vector<Eigen::Vector3f> targetSamplePoints;
    for (auto sample : samples) {
      sourceSamplePoints.push_back(sample.first);
      targetSamplePoints.push_back(sample.second);
    }

    auto [transformation, scale] = pointSetAlignment(sourceSamplePoints, targetSamplePoints);

    int numInliers = 0;
    for (auto pointPair : sourceTargetPointPairs) {
      Eigen::Vector3f sourcePoint = pointPair.first;
      Eigen::Vector3f targetPoint = pointPair.first;

      Eigen::Vector3f transformedSourcePoint
        = scale * transformation.rotationMatrix() * sourcePoint + transformation.translation();

      float scaledDistance = (transformedSourcePoint - targetPoint).squaredNorm() / targetVariance;

      if (scaledDistance < inlierThreshold)
        numInliers++;
    }

    if (numInliers > maxNumInliers) {
      maxNumInliers = numInliers;
      bestModel = make_pair(transformation, scale);
    }

    cout << numInliers << ", ";
  }

  if (maxNumInliers == 0)
    RCLCPP_INFO(this->get_logger(), "WARNING: no inliers in ransac scale");

  return bestModel;
}

// Least-squares estimation of transformation parameters between two point patterns
// DOI: 10.1109/34.88573
// https://zpl.fi/aligning-point-patterns-with-kabsch-umeyama-algorithm/
tuple<Sophus::SE3f, float> OrbSlam3Wrapper::pointSetAlignment(
  vector<Eigen::Vector3f> sourcePoints, vector<Eigen::Vector3f> targetPoints) {
  // Center around centroid
  Eigen::Vector3f souceCentroid = Eigen::Vector3f::Zero();
  for (Eigen::Vector3f point : sourcePoints) {
    souceCentroid += point;
  }
  souceCentroid /= sourcePoints.size();
  for (Eigen::Vector3f point : sourcePoints) {
    point -= souceCentroid;
  }

  Eigen::Vector3f targetCentroid = Eigen::Vector3f::Zero();
  for (Eigen::Vector3f point : targetPoints) {
    targetCentroid += point;
  }
  targetCentroid /= targetPoints.size();
  for (Eigen::Vector3f point : targetPoints) {
    point -= targetCentroid;
  }

  // Compute variance of source
  float sourceVariance = 0;
  for (Eigen::Vector3f point : sourcePoints) {
    sourceVariance += point.squaredNorm();
  }
  sourceVariance /= sourcePoints.size();

  // Compute covariance matrix
  Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
  for (int i = 0; i < sourcePoints.size(); i++) {
    H += sourcePoints[i] * targetPoints[i].transpose();
  }
  H /= sourcePoints.size();

  // Compute SVD
  Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

  // Compute scale
  float s = static_cast<Eigen::Matrix3f>(svd.singularValues().asDiagonal()).trace() / sourceVariance;

  // Compute rotation
  Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

  if (R.determinant() < 0) {
    RCLCPP_INFO(this->get_logger(), "point set alignemt failed");
    return { Sophus::SE3f(), 1 };
  }

  // Compute translation
  Eigen::Vector3f t = targetCentroid - R * souceCentroid;

  Sophus::SE3f sourceToTarget(R, t);
  return { sourceToTarget, s };
}