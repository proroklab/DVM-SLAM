#include "peer.h"
#include "KeyFrame.h"
#include <interfaces/msg/loop_closure_triggers.hpp>
#include <rclcpp/node.hpp>

Peer::Peer(rclcpp::Node::SharedPtr rosNode, uint agentId)
  : agentId(agentId)
  , remoteSuccessfullyMerged(false)
  , localSuccessfullyMerged(false)
  , referenceKeyFrame(nullptr)
  , rosNode(rosNode) {

  newKeyFrameBowsPub = rosNode->create_publisher<interfaces::msg::NewKeyFrameBows>(
    "robot" + to_string(agentId) + "/new_key_frame_bows", 1);
  getCurrentMapClient
    = rosNode->create_client<interfaces::srv::GetCurrentMap>("robot" + to_string(agentId) + "/get_current_map");
  successfullyMergedPub = rosNode->create_publisher<interfaces::msg::SuccessfullyMerged>(
    "robot" + to_string(agentId) + "/successfully_merged", 1);
  newKeyFramesPub
    = rosNode->create_publisher<interfaces::msg::NewKeyFrames>("robot" + to_string(agentId) + "/new_key_frames", 1);
  loopClosureTriggersPub = rosNode->create_publisher<interfaces::msg::LoopClosureTriggers>(
    "robot" + to_string(agentId) + "/loop_closure_triggers", 1);
  getMapPointsClient
    = rosNode->create_client<interfaces::srv::GetMapPoints>("robot" + to_string(agentId) + "/get_map_points");
  isLostFromBaseMapPub = rosNode->create_publisher<interfaces::msg::IsLostFromBaseMap>(
    "robot" + to_string(agentId) + "/is_lost_from_base_map", 1);
}

uint Peer::getId() { return agentId; }
bool Peer::getRemoteSuccessfullyMerged() { return remoteSuccessfullyMerged; }
bool Peer::getLocalSuccessfullyMerged() { return localSuccessfullyMerged; }
set<boost::uuids::uuid> Peer::getSentKeyFrameUuids() { return sentKeyFrameUuids; }
set<boost::uuids::uuid> Peer::getSentKeyFrameBowUuids() { return sentKeyFrameBowUuids; }
set<boost::uuids::uuid> Peer::getSentLoopClosureTriggerUuids() { return sentLoopClosureTriggerUuids; }
ORB_SLAM3::KeyFrame* Peer::getReferenceKeyFrame() { return referenceKeyFrame; }

bool Peer::getIsLostFromBaseMap() { return isLostFromBaseMap; }
void Peer::setIsLostFromBaseMap(bool isLost) { this->isLostFromBaseMap = isLost; }

void Peer::setReferenceKeyFrame(ORB_SLAM3::KeyFrame* referenceKeyFrame) { this->referenceKeyFrame = referenceKeyFrame; }

void Peer::setRemoteSuccessfullyMerged(bool remoteSuccessfullyMerged) {
  this->remoteSuccessfullyMerged = remoteSuccessfullyMerged;
}
void Peer::setLocalSuccessfullyMerged(bool localSuccessfullyMerged) {
  this->localSuccessfullyMerged = localSuccessfullyMerged;
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