#pragma once

#include "KeyFrame.h"
#include "interfaces/msg/is_lost_from_base_map.hpp"
#include "interfaces/msg/new_key_frame_bows.hpp"
#include "interfaces/msg/new_key_frames.hpp"
#include "interfaces/msg/successfully_merged.hpp"
#include "interfaces/srv/get_current_map.hpp"
#include "interfaces/srv/get_map_points.hpp"
#include <boost/uuid/uuid.hpp>
#include <interfaces/msg/change_coordinate_frame.hpp>
#include <interfaces/msg/loop_closure_triggers.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

using namespace std;

class Peer {
public:
  Peer(rclcpp::Node::SharedPtr rosNode, uint agentId);

  uint getId();
  set<uint> getRemoteSuccessfullyMerged();
  bool isRemoteSuccessfullyMerged(uint agentId);
  set<boost::uuids::uuid> getSentKeyFrameUuids();
  set<boost::uuids::uuid> getSentKeyFrameBowUuids();
  set<boost::uuids::uuid> getSentLoopClosureTriggerUuids();
  set<boost::uuids::uuid> getSentMapPointUuids();
  ORB_SLAM3::KeyFrame* getReferenceKeyFrame();

  bool isLeadNodeInGroup();

  bool getIsLostFromBaseMap();
  void setIsLostFromBaseMap(bool isLost);

  void setReferenceKeyFrame(ORB_SLAM3::KeyFrame* referenceKeyFrame);
  void updateRemoteSuccessfullyMerged(uint peerAgentId, bool successfullyMerged);
  void addSentKeyFrameUuids(
    _Rb_tree_const_iterator<boost::uuids::uuid> first, _Rb_tree_const_iterator<boost::uuids::uuid> last);
  void addSentKeyFrameUuid(boost::uuids::uuid uuid);
  void addSentKeyFrameBowUuids(
    _Rb_tree_const_iterator<boost::uuids::uuid> first, _Rb_tree_const_iterator<boost::uuids::uuid> last);
  void addSentKeyFrameBowUuid(boost::uuids::uuid uuid);
  void addSentLoopClosureTriggerUuids(
    _Rb_tree_const_iterator<boost::uuids::uuid> first, _Rb_tree_const_iterator<boost::uuids::uuid> last);
  void addSentLoopClosureTriggerUuid(boost::uuids::uuid uuid);
  void addSentMapPointUuid(boost::uuids::uuid uuid);

  rclcpp::Publisher<interfaces::msg::NewKeyFrames>::SharedPtr newKeyFramesPub;
  rclcpp::Publisher<interfaces::msg::NewKeyFrameBows>::SharedPtr newKeyFrameBowsPub;
  rclcpp::Publisher<interfaces::msg::IsLostFromBaseMap>::SharedPtr isLostFromBaseMapPub;
  rclcpp::Publisher<interfaces::msg::LoopClosureTriggers>::SharedPtr loopClosureTriggersPub;
  rclcpp::Publisher<interfaces::msg::ChangeCoordinateFrame>::SharedPtr changeCoordinateFramePub;
  rclcpp::Client<interfaces::srv::GetCurrentMap>::SharedPtr getCurrentMapClient;
  rclcpp::Client<interfaces::srv::GetMapPoints>::SharedPtr getMapPointsClient;

protected:
  uint agentId;

  set<boost::uuids::uuid> sentKeyFrameUuids;
  set<boost::uuids::uuid> sentKeyFrameBowUuids;
  set<boost::uuids::uuid> sentLoopClosureTriggerUuids;
  set<boost::uuids::uuid> sentMapPointUuids;
  ORB_SLAM3::KeyFrame* referenceKeyFrame = nullptr;

  // The agents which this peer has merged with
  // Note that if agent1 is merged with agent2, that does not necessarily mean that agent2 is merged with agent1
  set<uint> successfullyMergedAgentIds;

  bool isLostFromBaseMap = false; // Peer is lost in its own map

  rclcpp::Node::SharedPtr rosNode;
};