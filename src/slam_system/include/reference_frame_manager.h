#include "sophus/sim3.hpp"

class ReferenceFrameManager {
public:
  ReferenceFrameManager(std::string node_name) {
    // Initialize origin to be rotated 90 degrees about the x-axis
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix << 1, 0, 0, 0, 0, 1, 0, -1, 0;
    Eigen::Quaterniond quaternion(rotation_matrix);
    Eigen::Vector3d translation(0, 0, 0);
    world_to_origin = Sophus::Sim3d(quaternion, translation);
    world_to_origin.setScale(1);
    origin_frame_id = node_name + "/origin";
    slam_system_frame_id = origin_frame_id;
  }

  void setParentFrame(uint parentAgentId, Sophus::Sim3d parentToCurrentTransform) {
    origin_frame_parent_id = "robot" + std::to_string(parentAgentId) + "/origin";
    slam_system_frame_id = origin_frame_parent_id;

    world_to_origin = world_to_origin * parentToCurrentTransform;
  }

  std::string world_frame_id = "/world";
  std::string origin_frame_parent_id = world_frame_id;
  std::string origin_frame_id = "";
  Sophus::Sim3d world_to_origin;
  std::string slam_system_frame_id = "";
  std::string imu_frame_id = "/imu";
  std::string cam_frame_id = "camera";

protected:
};