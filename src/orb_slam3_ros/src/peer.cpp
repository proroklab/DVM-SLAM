#include "peer.h"
#include "KeyFrame.h"
#include <interfaces/msg/detail/change_coordinate_frame__struct.hpp>
#include <interfaces/msg/loop_closure_triggers.hpp>
#include <interfaces/msg/map_to_attempt_merge.hpp>
#include <rclcpp/node.hpp>

Peer::Peer(rclcpp::Node::SharedPtr rosNode, uint agentId)
  : agentId(agentId)
  , referenceKeyFrame(nullptr)
  , rosNode(rosNode) {

  newKeyFrameBowsPub = rosNode->create_publisher<interfaces::msg::NewKeyFrameBows>(
    "robot" + to_string(agentId) + "/new_key_frame_bows", rclcpp::QoS(3));
  getCurrentMapClient
    = rosNode->create_client<interfaces::srv::GetCurrentMap>("robot" + to_string(agentId) + "/get_current_map");
  newKeyFramesPub = rosNode->create_publisher<interfaces::msg::NewKeyFrames>(
    "robot" + to_string(agentId) + "/new_key_frames", rclcpp::QoS(3));
  loopClosureTriggersPub = rosNode->create_publisher<interfaces::msg::LoopClosureTriggers>(
    "robot" + to_string(agentId) + "/loop_closure_triggers", rclcpp::QoS(3));
  getMapPointsClient
    = rosNode->create_client<interfaces::srv::GetMapPoints>("robot" + to_string(agentId) + "/get_map_points");
  isLostFromBaseMapPub = rosNode->create_publisher<interfaces::msg::IsLostFromBaseMap>(
    "robot" + to_string(agentId) + "/is_lost_from_base_map", rclcpp::QoS(3));
  changeCoordinateFramePub = rosNode->create_publisher<interfaces::msg::ChangeCoordinateFrame>(
    "robot" + to_string(agentId) + "/change_coordinate_frame", rclcpp::QoS(3));
  mapToAttemptMergePub = rosNode->create_publisher<interfaces::msg::MapToAttemptMerge>(
    "robot" + to_string(agentId) + "/map_to_attempt_merge", rclcpp::QoS(3));
}

uint Peer::getId() { return agentId; }
set<uint> Peer::getSuccessfullyMerged() { return successfullyMergedAgentIds; }
bool Peer::isSuccessfullyMerged(uint agentId) { return successfullyMergedAgentIds.count(agentId) != 0; }
set<boost::uuids::uuid> Peer::getSentKeyFrameUuids() { return sentKeyFrameUuids; }
set<boost::uuids::uuid> Peer::getSentKeyFrameBowUuids() { return sentKeyFrameBowUuids; }
set<boost::uuids::uuid> Peer::getSentLoopClosureTriggerUuids() { return sentLoopClosureTriggerUuids; }
set<boost::uuids::uuid> Peer::getSentMapPointUuids() { return sentMapPointUuids; }
ORB_SLAM3::KeyFrame* Peer::getReferenceKeyFrame() { return referenceKeyFrame; }

bool Peer::getIsLostFromBaseMap() { return isLostFromBaseMap; }
void Peer::setIsLostFromBaseMap(bool isLost) { this->isLostFromBaseMap = isLost; }

bool Peer::isLeadNodeInGroup() {
  for (uint remoteSuccessfullyMerged : successfullyMergedAgentIds) {
    if (remoteSuccessfullyMerged < agentId)
      return false;
  }

  return true;
}

void Peer::setReferenceKeyFrame(ORB_SLAM3::KeyFrame* referenceKeyFrame) { this->referenceKeyFrame = referenceKeyFrame; }

void Peer::updateSuccessfullyMerged(uint peerAgentId, bool successfullyMerged) {
  if (successfullyMerged) {
    successfullyMergedAgentIds.insert(peerAgentId);
  }
  else {
    successfullyMergedAgentIds.erase(peerAgentId);
  }
}

void Peer::addSentKeyFrameUuids(
  _Rb_tree_const_iterator<boost::uuids::uuid> first, _Rb_tree_const_iterator<boost::uuids::uuid> last) {
  sentKeyFrameUuids.insert(first, last);
}
void Peer::addSentKeyFrameUuid(boost::uuids::uuid uuid) { sentKeyFrameUuids.insert(uuid); }

void Peer::addSentKeyFrameBowUuids(
  _Rb_tree_const_iterator<boost::uuids::uuid> first, _Rb_tree_const_iterator<boost::uuids::uuid> last) {
  sentKeyFrameBowUuids.insert(first, last);
}

void Peer::addSentKeyFrameBowUuid(boost::uuids::uuid uuid) { sentKeyFrameBowUuids.insert(uuid); }

void Peer::addSentLoopClosureTriggerUuids(
  _Rb_tree_const_iterator<boost::uuids::uuid> first, _Rb_tree_const_iterator<boost::uuids::uuid> last) {
  sentLoopClosureTriggerUuids.insert(first, last);
}

void Peer::addSentLoopClosureTriggerUuid(boost::uuids::uuid uuid) { sentLoopClosureTriggerUuids.insert(uuid); }

void Peer::addSentMapPointUuid(boost::uuids::uuid uuid) { sentMapPointUuids.insert(uuid); }