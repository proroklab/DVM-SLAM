#include "System.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "orb_slam3_wrapper.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std;

class OrbSlam3Mono : public OrbSlam3Wrapper {
public:
  OrbSlam3Mono(string voc_file, string settings_file)
    : OrbSlam3Wrapper("orb_slam3_mono", voc_file, settings_file, ORB_SLAM3::System::MONOCULAR) {

    image_subscriber
      = this->create_subscription<sensor_msgs::msg::Image>("robot" + to_string(agentId) + "/camera/image_color", 1,
        std::bind(&OrbSlam3Mono::grab_image, this, std::placeholders::_1));
  };

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
  rclcpp::Client<interfaces::srv::AddMap>::SharedPtr add_map_client;
  rclcpp::TimerBase::SharedPtr timer_;

  void grab_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // image must be copied since it uses the conversion_mat_ for storage
      // which is asynchronously overwritten in the next callback invocation
      cv::Mat image_cpy = cv_ptr->image.clone();

      rclcpp::Time timestamp = msg->header.stamp;
      double seconds = timestamp.nanoseconds() * 1.e-9;
      // RCLCPP_INFO(this->get_logger(), "New Image. Timestamp: %f", seconds);
      Sophus::SE3f Tcw = pSLAM->TrackMonocular(image_cpy, seconds);

      rclcpp::Time msg_time = msg->header.stamp;

      OrbSlam3Wrapper::publish_topics(msg_time);

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  string voc_file = "/home/joshuabird/Desktop/Parallels\ Shared\ Folders/ubuntuSharedFolder/"
                    "part_II_project/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt";
  string settings_file = "/home/joshuabird/Desktop/Parallels\ Shared\ Folders/"
                         "ubuntuSharedFolder/part_II_project/src/webots_sim/"
                         "worlds/webots.yaml";
  auto node = std::make_shared<OrbSlam3Mono>(voc_file, settings_file);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}