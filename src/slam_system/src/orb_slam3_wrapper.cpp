#include "orb_slam3_wrapper.h"

#include "DBoW2/DBoW2/BowVector.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "System.h"
#include "peer.h"
#include "sophus/se3.hpp"
#include "sophus/sim3.hpp"
#include "sophus/types.hpp"
#include <Eigen/src/Core/Matrix.h>
#include <Eigen/src/Geometry/Quaternion.h>
#include <algorithm>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <cstddef>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <interfaces/msg/change_coordinate_frame.hpp>
#include <interfaces/msg/map_point.hpp>
#include <interfaces/msg/map_to_attempt_merge.hpp>
#include <interfaces/msg/sim3_transform_stamped.hpp>
#include <interfaces/msg/successfully_merged.hpp>
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
#define MIN_BOW_SHARE_SIZE 5
#define MIN_MAP_POINTS_FOR_SCALE_ADJUSTMENT 500
#define RELIABLE_QOS rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_services_default)

using namespace std;

OrbSlam3Wrapper::OrbSlam3Wrapper(string node_name, ORB_SLAM3::System::eSensor sensor_type)
  : Node("robot" + to_string(agentId))
  , node_handle_(std::shared_ptr<OrbSlam3Wrapper>(this, [](auto*) {})) {
  // Create parameters
  this->declare_parameter("agentId", 1);
  agentId = this->get_parameter("agentId").as_int();

  this->declare_parameter("config",
    "/home/joshuabird/Desktop/Parallels\ Shared\ Folders/"
    "ubuntuSharedFolder/part_II_project/src/webots_sim/"
    "worlds/webots.yaml");
  settings_file = this->get_parameter("config").as_string();

  this->declare_parameter("vocFile",
    "/home/joshuabird/Desktop/Parallels\ Shared\ Folders/ubuntuSharedFolder/part_II_project/src/slam_system/"
    "orb_slam3/Vocabulary/ORBvoc.txt");
  voc_file = this->get_parameter("vocFile").as_string();

  this->declare_parameter("useViewer", true);
  bool useViewer = this->get_parameter("useViewer").as_bool();

  this->declare_parameter("publishVizTopics", true);
  publishVizTopics = this->get_parameter("publishVizTopics").as_bool();

  node_name = "robot" + to_string(agentId);
  RCLCPP_INFO(this->get_logger(), node_name.c_str());

  this->sensor_type = sensor_type;
  pSLAM = new ORB_SLAM3::System(voc_file, settings_file, sensor_type, agentId, useViewer);

  baseMap = pSLAM->GetAtlas()->GetCurrentMap();

  // Create publishers
  successfullyMergedPub
    = this->create_publisher<interfaces::msg::SuccessfullyMerged>("/successfully_merged", RELIABLE_QOS);

  // Create services
  get_current_map_service = this->create_service<interfaces::srv::GetCurrentMap>(node_name + "/get_current_map",
    std::bind(&OrbSlam3Wrapper::handleGetCurrentMapRequest, this, std::placeholders::_1, std::placeholders::_2));
  getMapPointsService = this->create_service<interfaces::srv::GetMapPoints>(node_name + "/get_map_points",
    std::bind(&OrbSlam3Wrapper::handleGetMapPointsRequest, this, std::placeholders::_1, std::placeholders::_2));

#ifdef RCL_SERVICE_INTROSPECTION_CONTENTS
  get_current_map_service->configure_introspection(this->get_clock(), RELIABLE_QOS, RCL_SERVICE_INTROSPECTION_CONTENTS);
  getMapPointsService->configure_introspection(this->get_clock(), RELIABLE_QOS, RCL_SERVICE_INTROSPECTION_CONTENTS);
#endif

  // Create subscriptions
  newKeyFrameBowsSub = this->create_subscription<interfaces::msg::NewKeyFrameBows>(node_name + "/new_key_frame_bows",
    RELIABLE_QOS, std::bind(&OrbSlam3Wrapper::receiveNewKeyFrameBows, this, std::placeholders::_1));
  successfullyMergedSub = this->create_subscription<interfaces::msg::SuccessfullyMerged>("/successfully_merged",
    RELIABLE_QOS, std::bind(&OrbSlam3Wrapper::receiveSuccessfullyMergedMsg, this, std::placeholders::_1));
  newKeyFramesSub = this->create_subscription<interfaces::msg::NewKeyFrames>(node_name + "/new_key_frames",
    RELIABLE_QOS, std::bind(&OrbSlam3Wrapper::receiveNewKeyFrames, this, std::placeholders::_1));
  isLostFromBaseMapSub
    = this->create_subscription<interfaces::msg::IsLostFromBaseMap>(node_name + "/is_lost_from_base_map", RELIABLE_QOS,
      std::bind(&OrbSlam3Wrapper::receiveIsLostFromBaseMapMsg, this, std::placeholders::_1));
  loopClosureTriggersSub
    = this->create_subscription<interfaces::msg::LoopClosureTriggers>(node_name + "/loop_closure_triggers",
      RELIABLE_QOS, std::bind(&OrbSlam3Wrapper::receiveLoopClosureTriggers, this, std::placeholders::_1));
  changeCoordinateFrameSub
    = this->create_subscription<interfaces::msg::ChangeCoordinateFrame>(node_name + "/change_coordinate_frame",
      RELIABLE_QOS, std::bind(&OrbSlam3Wrapper::receiveChangeCoordinateFrame, this, std::placeholders::_1));
  mapToAttemptMergeSub
    = this->create_subscription<interfaces::msg::MapToAttemptMerge>(node_name + "/map_to_attempt_merge", RELIABLE_QOS,
      std::bind(&OrbSlam3Wrapper::receiveMapToAttemptMerge, this, std::placeholders::_1));

  // TODO: create a proper topic handler that handles nodes connecting/disconnecting
  vector<uint> connectedPeerIds;
  if (agentId == 1)
    connectedPeerIds = { 2, 3 };
  else if (agentId == 2)
    connectedPeerIds = { 1, 3 };
  else if (agentId == 3)
    connectedPeerIds = { 1, 2 };

  for (uint connectedPeerId : connectedPeerIds) {
    connectedPeers[connectedPeerId] = new Peer(this->shared_from_this(), connectedPeerId);
  }

  updateScaleTimer = this->create_wall_timer(5s, std::bind(&OrbSlam3Wrapper::updateMapScale, this));

  referenceFrameManager = new ReferenceFrameManager(node_name);
  publishRosVizTopics = new PublishRosVizTopics(node_handle_, node_name, sensor_type, agentId, referenceFrameManager);

  publishRosVizTopics->resetVisualization();
};

void OrbSlam3Wrapper::run() {
  while (true) {
    if (newFrameProcessed) {
      updateSuccessfullyMerged();
      updateIsLostFromBaseMap();
      sendNewKeyFrameBows();
      sendNewKeyFrames();
      if (publishVizTopics)
        publish_topics(lastFrameTimestamp);

      newFrameProcessed = false;
    }

    rclcpp::spin_some(this->node_handle_);

    usleep(3000);
  }
}

void OrbSlam3Wrapper::handleGetCurrentMapRequest(const std::shared_ptr<interfaces::srv::GetCurrentMap::Request> request,
  std::shared_ptr<interfaces::srv::GetCurrentMap::Response> response) {
  startTimer("handleGetCurrentMapRequest");
  unique_lock<mutex> lock(mutexWrapper);
  unique_lock<mutex> lock2(pSLAM->GetAtlas()->GetCurrentMap()->mMutexMapUpdate); // Wait for map merge

  RCLCPP_INFO(this->get_logger(), "Handling get current map request");

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
  stopTimer("handleGetCurrentMapRequest");
}

void OrbSlam3Wrapper::handleGetCurrentMapResponse(rclcpp::Client<interfaces::srv::GetCurrentMap>::SharedFuture future) {
  startTimer("handleGetCurrentMapResponse");
  interfaces::srv::GetCurrentMap::Response::SharedPtr response = future.get();

  interfaces::msg::MapToAttemptMerge::SharedPtr mapToAttemptMergeMsg(new interfaces::msg::MapToAttemptMerge());

  RCLCPP_INFO(this->get_logger(), "Handling get current map response. Received serialized map. Size: %d",
    response->serialized_map.size());

  mapToAttemptMergeMsg->sender_agent_id = response->sender_agent_id;
  mapToAttemptMergeMsg->serialized_map = response->serialized_map;
  mapToAttemptMergeMsg->merge_candidate_key_frame_uuids = response->merge_candidate_key_frame_uuids;

  receiveMapToAttemptMerge(mapToAttemptMergeMsg);
  stopTimer("handleGetCurrentMapResponse");
}

void OrbSlam3Wrapper::receiveMapToAttemptMerge(interfaces::msg::MapToAttemptMerge::SharedPtr msg) {
  startTimer("receiveMapToAttemptMerge");
  unique_lock<mutex> lock(mutexWrapper);

  updateSuccessfullyMerged(true);
  if (isSuccessfullyMerged(msg->sender_agent_id) || !pSLAM->GetLoopCloser()->isFinishedGBA())
    return;

  // Get mergeCandidateKeyFrameUuids, which was generated by us when we created the getCurrentMap request
  vector<boost::uuids::uuid> mergeCandidateKeyFrameUuids;
  for (interfaces::msg::Uuid uuidMsg : msg->merge_candidate_key_frame_uuids) {
    mergeCandidateKeyFrameUuids.push_back(arrayToUuid(uuidMsg.uuid));
  }

  RCLCPP_INFO(this->get_logger(), "Recived map to attempt merge msg. Attempting to merge map. Size: %d",
    msg->serialized_map.size());

  pSLAM->AddSerializedMapToTryMerge(msg->serialized_map, mergeCandidateKeyFrameUuids);
  stopTimer("receiveMapToAttemptMerge");
}

void OrbSlam3Wrapper::sendNewKeyFrames() {
  startTimer("sendNewKeyFrames");
  unique_lock<mutex> lock(mutexWrapper);

  if (isLostFromBaseMap)
    return;

  // Send new key frames to all peers
  for (auto& pair : connectedPeers) {
    Peer* connectedPeer = pair.second;
    set<boost::uuids::uuid> sentKeyFrameUuids = connectedPeer->getSentKeyFrameUuids();
    set<boost::uuids::uuid> sentMapPointUuids = connectedPeer->getSentMapPointUuids();

    if (!isSuccessfullyMerged(connectedPeer->getId()) || connectedPeer->getIsLostFromBaseMap()) {
      continue;
    }

    startTimer("1");
    unsigned long largestKeyFrameId = 4;
    for (ORB_SLAM3::KeyFrame* keyFrame : pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames()) {
      if (keyFrame->mnId > largestKeyFrameId) {
        largestKeyFrameId = keyFrame->mnId;
      }
    }
    stopTimer("1");

    startTimer("2");
    // Check if we have enough keyframes to send update
    int newKeyFrames = 0;
    for (ORB_SLAM3::KeyFrame* keyFrame : pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames()) {
      if (sentKeyFrameUuids.count(keyFrame->uuid) == 0 && keyFrame->creatorAgentId == agentId && !keyFrame->isBad()
        && keyFrame->mnId < largestKeyFrameId - 3)
        newKeyFrames++;
    }
    if (newKeyFrames < MIN_KEY_FRAME_SHARE_SIZE)
      continue;
    stopTimer("2");

    startTimer("3");

    unique_ptr<ORB_SLAM3::Map> currentMapCopy = deepCopyMap(pSLAM->GetAtlas()->GetCurrentMap());

    largestKeyFrameId = 4;
    for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
      if (keyFrame->mnId > largestKeyFrameId) {
        largestKeyFrameId = keyFrame->mnId;
      }
    }
    stopTimer("3");

    startTimer("4");

    // Remove keyframes not from this agent or have already been sent
    // keep all keyframe connections to reconnect later
    vector<ORB_SLAM3::KeyFrame*> keyFramesToDelete;
    vector<ORB_SLAM3::KeyFrame*> keyFramesToBeSent;
    int a = currentMapCopy->GetMaxKFid();
    for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
      if (keyFrame->creatorAgentId != agentId
        || sentKeyFrameUuids.count(keyFrame->uuid) != 0
        // only send keyframes once they are outside of the mappoint culling window
        || keyFrame->isBad() || keyFrame->mnId >= largestKeyFrameId - 3) {
        currentMapCopy->EraseKeyFrame(keyFrame);
        keyFramesToDelete.push_back(keyFrame);
      }
      else {
        keyFramesToBeSent.push_back(keyFrame);
      }
    }
    stopTimer("4");

    startTimer("5");

    cout << "keyFramesToBeSent.size(): " << keyFramesToBeSent.size() << "\n";
    cout << "allKeyFrames.size(): " << currentMapCopy->GetAllKeyFrames().size() << "\n";

    // Remove mappoints that have already been sent, or who dont have a reference
    set<ORB_SLAM3::KeyFrame*> keyFramesToBeSentSet(keyFramesToBeSent.begin(), keyFramesToBeSent.end());
    for (ORB_SLAM3::MapPoint* mapPoint : currentMapCopy->GetAllMapPoints()) {
      if (mapPoint->creatorAgentId != agentId || sentMapPointUuids.count(mapPoint->uuid) != 0 || mapPoint->isBad()
        || (int)mapPoint->mnFirstKFid >= largestKeyFrameId - 3
        || (sentKeyFrameUuids.count(mapPoint->GetReferenceKeyFrame()->uuid) == 0
          && keyFramesToBeSentSet.count(mapPoint->GetReferenceKeyFrame()) == 0)) {
        mapPoint->SetBadFlag();
        delete mapPoint;
      }
    }
    stopTimer("5");

    startTimer("6");

#ifdef USE_REF_KEY_FRAMES
    // Get refernece key frame
    ORB_SLAM3::KeyFrame* referenceKeyFrame = connectedPeer->getReferenceKeyFrame();
    // If no reference keyframe, get the latest sent keyframe
    if (referenceKeyFrame == nullptr) {
      for (ORB_SLAM3::KeyFrame* keyFrame : pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames()) {
        if (connectedPeer->getSentKeyFrameUuids().count(keyFrame->uuid) != 0
          && (referenceKeyFrame == nullptr || keyFrame->mnId > referenceKeyFrame->mnId)) {
          referenceKeyFrame = keyFrame;
        }
      }
    }

    // Transform keyframes and map points to move the reference keyframe to the origin
    Sophus::SE3f referenceKeyFramePoseInv = referenceKeyFrame->GetPoseInverse();

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
#endif

    // Add keyframes to sent keyframes map
    for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
      connectedPeer->addSentKeyFrameUuid(keyFrame->uuid);
    }

    // Add map points to sent map points map
    for (ORB_SLAM3::MapPoint* mapPoint : currentMapCopy->GetAllMapPoints()) {
      connectedPeer->addSentMapPointUuid(mapPoint->uuid);
    }
    stopTimer("6");

    startTimer("7");

    // Get the lastest keyframe to later make as the reference keyframe uuid
    ORB_SLAM3::KeyFrame* latestKeyFrame = nullptr;
    for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
      if (!latestKeyFrame || (keyFrame->mnId > latestKeyFrame->mnId && keyFrame->creatorAgentId == agentId)) {
        // Get KF from current map, because this keyframe has had its coordinate frame changed
        latestKeyFrame = pSLAM->GetKeyFrameDatabase()->ConvertUuidToKeyFrame(keyFrame->uuid);
      }
    }
    ORB_SLAM3::KeyFrame* nextReferenceKeyFrame = latestKeyFrame;
    stopTimer("7");

    startTimer("8");

    // Send new frames to peer
    interfaces::msg::NewKeyFrames msg;
    msg.header.stamp = lastFrameTimestamp;
    msg.sender_agent_id = agentId;
    msg.serialized_map = pSLAM->SerializeMap(currentMapCopy.get());
#ifdef USE_REF_KEY_FRAMES
    msg.reference_key_frame_uuid = uuidToArray(referenceKeyFrame->uuid);
#endif
    msg.next_reference_key_frame_uuid = uuidToArray(nextReferenceKeyFrame->uuid);
    stopTimer("8");

    startTimer("9");
    connectedPeer->newKeyFramesPub->publish(msg);

    // Actually set referenceKeyFrameUuid
    connectedPeer->setReferenceKeyFrame(nextReferenceKeyFrame);

    for (ORB_SLAM3::KeyFrame* keyFrame : keyFramesToDelete) { // TODO: REALLY REALLY should move to smart pointers
      if (keyFrame)
        delete keyFrame;
    }
    stopTimer("9");

    RCLCPP_INFO(this->get_logger(), "Sent new key frames");
  }
  stopTimer("sendNewKeyFrames");
}

void OrbSlam3Wrapper::receiveNewKeyFrames(const interfaces::msg::NewKeyFrames::SharedPtr msg) {
  startTimer("receiveNewKeyFrames");
  unique_lock<mutex> lock(mutexWrapper);

  RCLCPP_INFO(this->get_logger(), "Received new key frames. Size: %d", msg->serialized_map.size());

  ORB_SLAM3::Map* newMap = pSLAM->GetAtlas()->DeserializeMap(msg->serialized_map, true);
  ORB_SLAM3::KeyFrame* referenceKeyFrame
    = pSLAM->GetKeyFrameDatabase()->ConvertUuidToKeyFrame(arrayToUuid(msg->reference_key_frame_uuid));

  {
    unique_lock<mutex> lock2(pSLAM->GetAtlas()->GetCurrentMap()->mMutexMapUpdate);

#ifdef USE_REF_KEY_FRAMES
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
#endif
  }

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
  vector<ORB_SLAM3::KeyFrame*> sortedKeyFrames = newMap->GetAllKeyFrames();
  sort(sortedKeyFrames.begin(), sortedKeyFrames.end(),
    [](ORB_SLAM3::KeyFrame* a, ORB_SLAM3::KeyFrame* b) { return a->mnId < b->mnId; });
  for (ORB_SLAM3::KeyFrame* keyFrame : sortedKeyFrames) {
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
  stopTimer("receiveNewKeyFrames");
}

void OrbSlam3Wrapper::sendNewKeyFrameBows() {
  startTimer("sendNewKeyFrameBows");
  unique_lock<mutex> lock(mutexWrapper);

  if (isLostFromBaseMap)
    return;

  vector<ORB_SLAM3::KeyFrame*> keyFrames = pSLAM->GetAllKeyFrames();

  if (keyFrames.size() < 12) // need at least 12 KFs to do a map merge
    return;

  // Send new key frame bows to all peers
  for (auto& pair : connectedPeers) {
    Peer* connectedPeer = pair.second;

    if (isSuccessfullyMerged(connectedPeer->getId()) || connectedPeer->getIsLostFromBaseMap()) {
      continue;
    }

    // Only send bow to peer if it is the lead node in its group and we are lead node in group
    // ie. we both have the lowest agentId of all its connected peers
    // if (!connectedPeer->isLeadNodeInGroup() || !isLeadNodeInGroup())
    //   continue;

    // Check if we have enough keyframes to send update
    int newKeyFrames = 0;
    for (ORB_SLAM3::KeyFrame* keyFrame : keyFrames) {
      if ((keyFrame->creatorAgentId == agentId || isSuccessfullyMerged(keyFrame->creatorAgentId))
        && connectedPeer->getSentKeyFrameBowUuids().count(keyFrame->uuid) == 0)
        newKeyFrames++;
    }
    if (newKeyFrames < MIN_BOW_SHARE_SIZE)
      continue;

    set<boost::uuids::uuid> newKeyFrameBowUuids;
    vector<interfaces::msg::KeyFrameBowVector> keyFrameBowVectorMsgs;

    for (ORB_SLAM3::KeyFrame* keyFrame : keyFrames) {
      if ((keyFrame->creatorAgentId == agentId || isSuccessfullyMerged(keyFrame->creatorAgentId))
        && connectedPeer->getSentKeyFrameBowUuids().count(keyFrame->uuid) == 0) {
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

    if (keyFrameBowVectorMsgs.size() == 0)
      return;

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
  stopTimer("sendNewKeyFrameBows");
}

void OrbSlam3Wrapper::receiveNewKeyFrameBows(const interfaces::msg::NewKeyFrameBows::SharedPtr msg) {
  startTimer("receiveNewKeyFrameBows");
  unique_lock<mutex> lock(mutexWrapper);

  vector<interfaces::msg::KeyFrameBowVector> newKeyFramesBowVectors = msg->key_frame_bow_vectors;
  RCLCPP_INFO(this->get_logger(), "received new key frame bow vectors");

  // Only attempt merge if we have the lowest agent id of all connected peers
  if (!isLeadNodeInGroup())
    return;

  if (isSuccessfullyMerged(msg->sender_agent_id)) {
    return;
  }

  if (pSLAM->GetAllKeyFrames().size() < 12) // need at least 12 KFs to do a map merge
    return;

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

    vector<interfaces::msg::Uuid> mergeCandidateKeyFrameUuidMsgs;
    for (boost::uuids::uuid uuid : mergeCandidateKeyFrameUuids) {
      interfaces::msg::Uuid uuidMsg;
      uuidMsg.uuid = uuidToArray(uuid);
      mergeCandidateKeyFrameUuidMsgs.push_back(uuidMsg);
    }

    // The agent with the larger agent id attempts to merge with the other agents map
    if (agentId > msg->sender_agent_id) {
      RCLCPP_INFO(this->get_logger(), "Merge with peer possible! Requesting its full map");

      auto request = std::make_shared<interfaces::srv::GetCurrentMap::Request>();
      request->sender_agent_id = agentId;
      request->merge_candidate_key_frame_uuids
        = mergeCandidateKeyFrameUuidMsgs; // Keep this state with the request for us to use later
      auto future_result = connectedPeers[msg->sender_agent_id]->getCurrentMapClient->async_send_request(
        request, bind(&OrbSlam3Wrapper::handleGetCurrentMapResponse, this, placeholders::_1));
    }
    else {
      RCLCPP_INFO(this->get_logger(), "Merge with peer possible! Sending it our full map");
      unique_lock<mutex> lock2(pSLAM->GetAtlas()->GetCurrentMap()->mMutexMapUpdate); // Wait for map merge

      // Clone current map
      unique_ptr<ORB_SLAM3::Map> currentMapCopy = deepCopyMap(pSLAM->GetAtlas()->GetCurrentMap());

      // Remove keyframes not from this agent
      for (ORB_SLAM3::KeyFrame* keyFrame : currentMapCopy->GetAllKeyFrames()) {
        if (keyFrame->creatorAgentId != agentId) {
          keyFrame->SetBadFlag(true);
        }
      }

      interfaces::msg::MapToAttemptMerge mapToAttemptMergeMsg;
      mapToAttemptMergeMsg.sender_agent_id = agentId;
      mapToAttemptMergeMsg.merge_candidate_key_frame_uuids = mergeCandidateKeyFrameUuidMsgs;
      mapToAttemptMergeMsg.serialized_map = pSLAM->SerializeMap(currentMapCopy.get());

      connectedPeers[msg->sender_agent_id]->mapToAttemptMergePub->publish(mapToAttemptMergeMsg);
    }
  }
  stopTimer("receiveNewKeyFrameBows");
}

void OrbSlam3Wrapper::updateSuccessfullyMerged(bool mutexWrapperLockAlreadyHeld) {
  startTimer("updateSuccessfullyMerged");
  unique_lock<mutex> lock;
  if (!mutexWrapperLockAlreadyHeld) {
    lock = unique_lock<mutex>(mutexWrapper);
  }

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

    if (successfullyMerged && isSuccessfullyMerged(connectedPeer->getId()) != successfullyMerged
      && pSLAM->GetLoopCloser()->isFinishedGBA()) {
      baseMap = pSLAM->GetAtlas()->GetCurrentMap();

      vector<boost::uuids::uuid> mergedKeyFrameUuid = successfullyMergedAgentIds[connectedPeer->getId()].first;
      Sophus::Sim3d mergeWorldToCurrentWorld = successfullyMergedAgentIds[connectedPeer->getId()].second;

      connectedPeer->updateSuccessfullyMerged(agentId, successfullyMerged);

      // Redefine our reference frame if we are moving to the other agents reference frame
      if (connectedPeer->getId() < agentId) {
        referenceFrameManager->setParentFrame(connectedPeer->getId(), mergeWorldToCurrentWorld);
      }

      // Tell the connected peers to change their coordinate frame too
      sendChangeCoordinateFrame(connectedPeer->getId(), mergeWorldToCurrentWorld);

      RCLCPP_INFO(this->get_logger(),
        ("Successfully merged with peer " + to_string(connectedPeer->getId()) + " "
          + (successfullyMerged ? "true" : "false"))
          .c_str());

      // Tell this agent that we are successfully merged
      interfaces::msg::SuccessfullyMerged msg;
      msg.header.stamp = lastFrameTimestamp;
      msg.successfully_merged = successfullyMerged;
      msg.sender_agent_id = agentId;
      msg.implicit_merge = false;
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
      msg.receiver_agent_id = connectedPeer->getId();
      successfullyMergedPub->publish(msg);

      // Tell all of the agents connected peers that we are implicitly merged with them
      for (uint peerAgentId : connectedPeer->getSuccessfullyMerged()) {
        if (peerAgentId != agentId && !isSuccessfullyMerged(peerAgentId)) {
          connectedPeers[peerAgentId]->updateSuccessfullyMerged(agentId, true);

          interfaces::msg::SuccessfullyMerged msg;
          msg.header.stamp = lastFrameTimestamp;
          msg.successfully_merged = true;
          msg.sender_agent_id = agentId;
          msg.implicit_merge = true;
          msg.receiver_agent_id = peerAgentId;

          for (ORB_SLAM3::KeyFrame* keyFrame : pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames()) {
            if (keyFrame->creatorAgentId == peerAgentId) {
              interfaces::msg::Uuid uuidMsg;
              uuidMsg.uuid = uuidToArray(keyFrame->uuid);
              msg.all_key_frames_in_map.push_back(uuidMsg);
            }
          }

          successfullyMergedPub->publish(msg);
        }
      }
    }
  }
  stopTimer("updateSuccessfullyMerged");
}

void OrbSlam3Wrapper::receiveSuccessfullyMergedMsg(const interfaces::msg::SuccessfullyMerged::SharedPtr msg) {
  startTimer("receiveSuccessfullyMergedMsg");
  unique_lock<mutex> lock(mutexWrapper);

  if (msg->sender_agent_id != agentId)
    connectedPeers[msg->sender_agent_id]->updateSuccessfullyMerged(msg->receiver_agent_id, msg->successfully_merged);

  if (msg->receiver_agent_id != agentId)
    connectedPeers[msg->receiver_agent_id]->updateSuccessfullyMerged(msg->sender_agent_id, msg->successfully_merged);

  if (msg->receiver_agent_id == agentId) {
    // Add existing keyframes to sent keyframes map
    set<boost::uuids::uuid> sentKeyFrameUuids;
    for (interfaces::msg::Uuid uuidMsg : msg->all_key_frames_in_map)
      sentKeyFrameUuids.insert(arrayToUuid(uuidMsg.uuid));

    connectedPeers[msg->sender_agent_id]->addSentKeyFrameUuids(sentKeyFrameUuids.begin(), sentKeyFrameUuids.end());
  }
  stopTimer("receiveSuccessfullyMergedMsg");
}

void OrbSlam3Wrapper::updateIsLostFromBaseMap() {
  startTimer("updateIsLostFromBaseMap");
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
  stopTimer("updateIsLostFromBaseMap");
}

void OrbSlam3Wrapper::receiveIsLostFromBaseMapMsg(const interfaces::msg::IsLostFromBaseMap::SharedPtr msg) {
  startTimer("receiveIsLostFromBaseMapMsg");
  unique_lock<mutex> lock(mutexWrapper);

  connectedPeers[msg->sender_agent_id]->setIsLostFromBaseMap(msg->is_lost_from_base_map);
  stopTimer("receiveIsLostFromBaseMapMsg");
}

void OrbSlam3Wrapper::updateMapScale() {
  startTimer("updateMapScale");
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

    auto [transformation, scale] = ransacPointSetAlignment(sourcePoints, targetPoints, 4, 500, 1e-5);

    pSLAM->GetAtlas()->GetCurrentMap()->ApplyScaledRotation(transformation, scale);

    RCLCPP_INFO(this->get_logger(), "Applied transformation");
    cout << "translation: " << transformation.translation() << " rotation: " << transformation.rotationMatrix()
         << " & scale: " << scale;

    float threshold = 0.01;
    // Additive increase, multiplicative decrease
    if (scale - 1 < threshold && scale - 1 > -threshold) {
      mapAlignmentPreviousExponentialBackoffCounter++;
    }
    else {
      mapAlignmentPreviousExponentialBackoffCounter = mapAlignmentPreviousExponentialBackoffCounter / 2;
    }
    mapAlignmentExponentialBackoffCounter = mapAlignmentPreviousExponentialBackoffCounter;
  };

  if (mapAlignmentExponentialBackoffCounter > 0) {
    mapAlignmentExponentialBackoffCounter--;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Attempting to update map scale...");

  uint lowestConnectedPeerAgentId = connectedPeers.begin()->first;
  Peer* lowestConnectedPeer = connectedPeers[lowestConnectedPeerAgentId];

  if (agentId < lowestConnectedPeerAgentId || isSuccessfullyMerged(lowestConnectedPeer->getId()) == false
    || lowestConnectedPeer->getIsLostFromBaseMap()) {
    return;
  }

  auto request = std::make_shared<interfaces::srv::GetMapPoints::Request>();
  auto future_result = lowestConnectedPeer->getMapPointsClient->async_send_request(request, handleGetMapPointsResponse);
  stopTimer("updateMapScale");
}

void OrbSlam3Wrapper::handleGetMapPointsRequest(const std::shared_ptr<interfaces::srv::GetMapPoints::Request> request,
  std::shared_ptr<interfaces::srv::GetMapPoints::Response> response) {
  startTimer("handleGetMapPointsRequest");
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
  stopTimer("handleGetMapPointsRequest");
}

void OrbSlam3Wrapper::sendLoopClosureTriggers() {
  startTimer("sendLoopClosureTriggers");
  unique_lock<mutex> lock(mutexWrapper);

  if (isLostFromBaseMap)
    return;

  set<boost::uuids::uuid> loopClosureTriggers = pSLAM->GetAtlas()->GetLoopClosureTriggers();

  for (auto& pair : connectedPeers) {
    Peer* connectedPeer = pair.second;

    // If the peer has not successfully merged, we dont need to send loop closure triggers
    if (!isSuccessfullyMerged(connectedPeer->getId())) {
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
  stopTimer("sendLoopClosureTriggers");
}

void OrbSlam3Wrapper::receiveLoopClosureTriggers(const interfaces::msg::LoopClosureTriggers::SharedPtr msg) {
  startTimer("receiveLoopClosureTriggers");
  unique_lock<mutex> lock(mutexWrapper);

  RCLCPP_INFO(this->get_logger(), "Received loop closure triggers");

  for (interfaces::msg::Uuid uuidMsg : msg->trigger_key_frame_uuids) {
    boost::uuids::uuid uuid = arrayToUuid(uuidMsg.uuid);
    ORB_SLAM3::KeyFrame* keyFrame = pSLAM->GetKeyFrameDatabase()->ConvertUuidToKeyFrame(uuid);
    pSLAM->GetLoopCloser()->InsertKeyFrame(keyFrame, keyFrame->GetMap());
  }
  stopTimer("receiveLoopClosureTriggers");
}

void OrbSlam3Wrapper::sendChangeCoordinateFrame(uint parentCoordFrameAgentId, Sophus::Sim3d parentToCurrentTransform) {
  startTimer("sendChangeCoordinateFrame");
  RCLCPP_INFO(this->get_logger(), "Sent change coordinate frame messages");

  // Send to all connected peers apart from the peer we just connected to
  for (Peer* successfullyMergedPeer : getSuccessfullyMergedPeers()) {
    if (successfullyMergedPeer->getId() != parentCoordFrameAgentId) {
      interfaces::msg::ChangeCoordinateFrame msg;

      msg.header.stamp = lastFrameTimestamp;
      msg.sender_agent_id = agentId;
      msg.new_coord_frame_parent_agent_id = parentCoordFrameAgentId;
      geometry_msgs::msg::Quaternion quaternionMsg;
      quaternionMsg.w = parentToCurrentTransform.quaternion().w();
      quaternionMsg.x = parentToCurrentTransform.quaternion().x();
      quaternionMsg.y = parentToCurrentTransform.quaternion().y();
      quaternionMsg.z = parentToCurrentTransform.quaternion().z();
      msg.parent_to_current_transform.rotation = quaternionMsg;
      geometry_msgs::msg::Vector3 translationMsg;
      translationMsg.x = parentToCurrentTransform.translation().x();
      translationMsg.y = parentToCurrentTransform.translation().y();
      translationMsg.z = parentToCurrentTransform.translation().z();
      msg.parent_to_current_transform.translation = translationMsg;
      msg.parent_to_current_transform.scale = parentToCurrentTransform.scale();

      successfullyMergedPeer->changeCoordinateFramePub->publish(msg);
    }
  }
  stopTimer("sendChangeCoordinateFrame");
}

void OrbSlam3Wrapper::receiveChangeCoordinateFrame(const interfaces::msg::ChangeCoordinateFrame::SharedPtr msg) {
  startTimer("receiveChangeCoordinateFrame");
  unique_lock<mutex> lock(mutexWrapper);
  RCLCPP_INFO(this->get_logger(), "Received change coordinate frame msg");

  Eigen::Quaterniond quaternion(msg->parent_to_current_transform.rotation.w,
    msg->parent_to_current_transform.rotation.x, msg->parent_to_current_transform.rotation.y,
    msg->parent_to_current_transform.rotation.z);

  Eigen::Vector3d translation(msg->parent_to_current_transform.translation.x,
    msg->parent_to_current_transform.translation.y, msg->parent_to_current_transform.translation.z);

  Sophus::Sim3d parentToCurrentTransform(quaternion, translation);
  parentToCurrentTransform.setScale(msg->parent_to_current_transform.scale);

  referenceFrameManager->setParentFrame(msg->new_coord_frame_parent_agent_id, parentToCurrentTransform);

  Sophus::SE3f se3fTransformation(
    parentToCurrentTransform.quaternion().cast<float>(), parentToCurrentTransform.translation().cast<float>());

  pSLAM->GetAtlas()->GetCurrentMap()->ApplyScaledRotation(se3fTransformation, msg->parent_to_current_transform.scale);

  // Tell parent that we are successfully merged
  if (!isSuccessfullyMerged(msg->new_coord_frame_parent_agent_id)) {
    interfaces::msg::SuccessfullyMerged newMsg;
    newMsg.header.stamp = lastFrameTimestamp;
    newMsg.successfully_merged = true;
    newMsg.sender_agent_id = agentId;
    newMsg.implicit_merge = true;
    newMsg.receiver_agent_id = msg->new_coord_frame_parent_agent_id;

    successfullyMergedPub->publish(newMsg);
  }

  // Tell everyone in the parent's group that we are successfully merged
  for (uint peerAgentId : connectedPeers[msg->new_coord_frame_parent_agent_id]->getSuccessfullyMerged()) {
    if (!isSuccessfullyMerged(peerAgentId)) {
      interfaces::msg::SuccessfullyMerged msg;
      msg.header.stamp = lastFrameTimestamp;
      msg.successfully_merged = true;
      msg.sender_agent_id = agentId;
      msg.implicit_merge = true;
      msg.receiver_agent_id = peerAgentId;

      successfullyMergedPub->publish(msg);
    }
  }
  stopTimer("receiveChangeCoordinateFrame");
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
  startTimer("deepCopyMap");
  // Make a deep copy of the target map
  // TODO: make this less terrible!
  ORB_SLAM3::Map* mapCopy;

  startTimer("deepCopyMap 1");
  // HACKY: create dummy objects for post load
  vector<ORB_SLAM3::GeometricCamera*> mvpCameras = pSLAM->GetAtlas()->GetAllCameras();
  set<ORB_SLAM3::GeometricCamera*> dummySCams(mvpCameras.begin(), mvpCameras.end());
  ORB_SLAM3::ORBVocabulary* dummyORBVoc = pSLAM->GetORBVocabulary();
  if (dummyKFDB != nullptr)
    delete dummyKFDB;
  dummyKFDB = new ORB_SLAM3::KeyFrameDatabase(*dummyORBVoc);
  map<unsigned int, ORB_SLAM3::GeometricCamera*> dummyMCams;
  for (ORB_SLAM3::GeometricCamera* pCam : mvpCameras) {
    dummyMCams[pCam->GetId()] = pCam;
  }
  stopTimer("deepCopyMap 1");
  startTimer("deepCopyMap 2");

  // Copy using serialization / deserialization
  std::ostringstream oss;
  boost::archive::binary_oarchive oa(oss);
  targetMap->PreSave(dummySCams);
  stopTimer("deepCopyMap 2");
  startTimer("deepCopyMap 3");
  oa << targetMap;
  stopTimer("deepCopyMap 3");
  startTimer("deepCopyMap 4");
  std::istringstream iss(oss.str());
  boost::archive::binary_iarchive ia(iss);
  stopTimer("deepCopyMap 4");
  startTimer("deepCopyMap 5");
  ia >> mapCopy;
  stopTimer("deepCopyMap 5");
  startTimer("deepCopyMap 6");
  mapCopy->PostLoad(dummyKFDB, dummyORBVoc, dummyMCams);
  stopTimer("deepCopyMap 6");

  stopTimer("deepCopyMap");
  return unique_ptr<ORB_SLAM3::Map>(mapCopy);
}

void OrbSlam3Wrapper::publish_topics(rclcpp::Time msg_time, Eigen::Vector3f Wbb) {
  startTimer("publish_topics");
  // unique_lock<mutex> lock(mutexWrapper);

  Sophus::SE3f Twc = pSLAM->GetCamTwc();

  if (Twc.translation().array().isNaN()[0] || Twc.rotationMatrix().array().isNaN()(0, 0)) // avoid publishing NaN
    return;

  // Publish origin transformation
  Sophus::SE3d world_to_origin_se3d(
    referenceFrameManager->world_to_origin.quaternion(), referenceFrameManager->world_to_origin.translation());
  publishRosVizTopics->publish_tf_transform(world_to_origin_se3d, referenceFrameManager->origin_frame_parent_id,
    referenceFrameManager->origin_frame_id, msg_time);
  publishRosVizTopics->publish_sim3_transform(referenceFrameManager->world_to_origin,
    referenceFrameManager->origin_frame_parent_id, referenceFrameManager->origin_frame_id, msg_time);

  // Common topics
  // publish_tf_transform(Twc, origin_frame_id, cam_frame_id, msg_time);
  // publish_tracking_img(pSLAM->GetCurrentFrame(), msg_time);
  publishRosVizTopics->publish_tracked_points(pSLAM->GetTrackedMapPoints(), msg_time);
  publishRosVizTopics->publish_all_points(pSLAM->GetAllMapPoints(), msg_time);
  publishRosVizTopics->publish_keyframes(pSLAM->GetAtlas()->GetCurrentMap()->GetAllKeyFrames(), msg_time);

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
    // publishRosVizTopics->publish_body_odom(Twb, Vwb, Wwb, msg_time);
  }
  stopTimer("publish_topics");
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

bool OrbSlam3Wrapper::isSuccessfullyMerged(uint otherAgentId) {
  return otherAgentId != agentId && connectedPeers[otherAgentId]->isSuccessfullyMerged(agentId);
}

vector<Peer*> OrbSlam3Wrapper::getSuccessfullyMergedPeers() {
  vector<Peer*> successfullyMergedPeers;
  for (auto pair : connectedPeers) {
    auto connectedPeerId = pair.first;
    auto connectedPeer = pair.second;

    if (isSuccessfullyMerged(connectedPeerId))
      successfullyMergedPeers.push_back(connectedPeer);
  }

  return successfullyMergedPeers;
}

bool OrbSlam3Wrapper::isLeadNodeInGroup() {
  for (Peer* successfullyMergedPeer : getSuccessfullyMergedPeers()) {
    if (isSuccessfullyMerged(successfullyMergedPeer->getId()) && successfullyMergedPeer->getId() < agentId)
      return false;
  }
  return true;

  return true;
}