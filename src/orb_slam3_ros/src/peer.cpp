#include "peer.h"
#include "KeyFrame.h"
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
  getMapPointsClient
    = rosNode->create_client<interfaces::srv::GetMapPoints>("robot" + to_string(agentId) + "/get_map_points");
}

uint Peer::getId() { return agentId; }
bool Peer::getRemoteSuccessfullyMerged() { return remoteSuccessfullyMerged; }
bool Peer::getLocalSuccessfullyMerged() { return localSuccessfullyMerged; }
set<boost::uuids::uuid> Peer::getSentKeyFrameUuids() { return sentKeyFrameUuids; }
set<boost::uuids::uuid> Peer::getSentKeyFrameBowUuids() { return sentKeyFrameBowUuids; }
ORB_SLAM3::KeyFrame* Peer::getReferenceKeyFrame() { return referenceKeyFrame; }

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