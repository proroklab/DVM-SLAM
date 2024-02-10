#pragma once

#include "KeyFrame.h"
#include "interfaces/msg/is_lost_from_base_map.hpp"
#include "interfaces/msg/new_key_frame_bows.hpp"
#include "interfaces/msg/new_key_frames.hpp"
#include "interfaces/msg/successfully_merged.hpp"
#include "interfaces/srv/get_current_map.hpp"
#include "interfaces/srv/get_map_points.hpp"
#include <boost/uuid/uuid.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <rclcpp/subscription.hpp>

using namespace std;

class Peer {
public:
  Peer(rclcpp::Node::SharedPtr rosNode, uint agentId);

  uint getId();
  bool getRemoteSuccessfullyMerged();
  bool getLocalSuccessfullyMerged();
  set<boost::uuids::uuid> getSentKeyFrameUuids();
  set<boost::uuids::uuid> getSentKeyFrameBowUuids();
  ORB_SLAM3::KeyFrame* getReferenceKeyFrame();

  bool getIsLostFromBaseMap();
  void setIsLostFromBaseMap(bool isLost);

  void setReferenceKeyFrame(ORB_SLAM3::KeyFrame* referenceKeyFrame);
  void setRemoteSuccessfullyMerged(bool remoteSuccessfullyMerged);
  void setLocalSuccessfullyMerged(bool localSuccessfullyMerged);
  void addSentKeyFrameUuids(
    _Rb_tree_const_iterator<boost::uuids::uuid> first, _Rb_tree_const_iterator<boost::uuids::uuid> last);
  void addSentKeyFrameUuid(boost::uuids::uuid uuid);
  void addSentKeyFrameBowUuids(
    _Rb_tree_const_iterator<boost::uuids::uuid> first, _Rb_tree_const_iterator<boost::uuids::uuid> last);
  void addSentKeyFrameBowUuid(boost::uuids::uuid uuid);

  rclcpp::Publisher<interfaces::msg::NewKeyFrames>::SharedPtr newKeyFramesPub;
  rclcpp::Publisher<interfaces::msg::NewKeyFrameBows>::SharedPtr newKeyFrameBowsPub;
  rclcpp::Publisher<interfaces::msg::SuccessfullyMerged>::SharedPtr successfullyMergedPub;
  rclcpp::Publisher<interfaces::msg::IsLostFromBaseMap>::SharedPtr isLostFromBaseMapPub;
  rclcpp::Client<interfaces::srv::GetCurrentMap>::SharedPtr getCurrentMapClient;
  rclcpp::Client<interfaces::srv::GetMapPoints>::SharedPtr getMapPointsClient;

  vector<boost::uuids::uuid> mergeCandidateKeyFrameUuids;

protected:
  uint agentId;

  set<boost::uuids::uuid> sentKeyFrameUuids;
  set<boost::uuids::uuid> sentKeyFrameBowUuids;
  ORB_SLAM3::KeyFrame* referenceKeyFrame;

  bool remoteSuccessfullyMerged; // Remote map is successfully merged together
  bool localSuccessfullyMerged; // Local map is successfully merged together

  bool isLostFromBaseMap = false; // Peer is lost in its own map

  rclcpp::Node::SharedPtr rosNode;
};