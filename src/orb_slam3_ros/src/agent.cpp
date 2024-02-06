#include "agent.h"
#include "KeyFrame.h"
#include <rclcpp/node.hpp>

Agent::Agent(rclcpp::Node::SharedPtr rosNode, uint agentId)
  : agentId(agentId)
  , successfullyMerged(false)
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
}

uint Agent::getId() { return agentId; }
bool Agent::getSuccessfullyMerged() { return successfullyMerged; }
set<boost::uuids::uuid> Agent::getSentKeyFrameUuids() { return sentKeyFrameUuids; }
set<boost::uuids::uuid> Agent::getSentKeyFrameBowUuids() { return sentKeyFrameBowUuids; }
ORB_SLAM3::KeyFrame* Agent::getReferenceKeyFrame() { return referenceKeyFrame; }

void Agent::setReferenceKeyFrame(ORB_SLAM3::KeyFrame* referenceKeyFrame) {
  this->referenceKeyFrame = referenceKeyFrame;
}

void Agent::setSuccessfullyMerged(bool successfullyMerged) { this->successfullyMerged = successfullyMerged; }

void Agent::addSentKeyFrameUuids(
  _Rb_tree_const_iterator<boost::uuids::uuid> first, _Rb_tree_const_iterator<boost::uuids::uuid> last) {
  sentKeyFrameUuids.insert(first, last);
}
void Agent::addSentKeyFrameUuid(boost::uuids::uuid uuid) { sentKeyFrameUuids.insert(uuid); }

void Agent::addSentKeyFrameBowUuids(
  _Rb_tree_const_iterator<boost::uuids::uuid> first, _Rb_tree_const_iterator<boost::uuids::uuid> last) {
  sentKeyFrameBowUuids.insert(first, last);
}

void Agent::addSentKeyFrameBowUuid(boost::uuids::uuid uuid) { sentKeyFrameBowUuids.insert(uuid); }