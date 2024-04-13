#include "System.h"
#include "cv_bridge/cv_bridge.h" // IWYU pragma: keep
#include "opencv2/highgui/highgui.hpp"
#include "orb_slam3_wrapper.h"
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_overriding_options.hpp>
#include <rmw/qos_profiles.h>
#include <rmw/types.h>
#include <utility>

#define BEST_EFFORT_QOS rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_sensor_data)
#define RELIABLE_QOS rclcpp::QoS(rclcpp::KeepLast(10), rmw_qos_profile_services_default)

using namespace std;

class OrbSlam3Mono : public OrbSlam3Wrapper {
public:
  OrbSlam3Mono()
    : OrbSlam3Wrapper("orb_slam3_mono", ORB_SLAM3::System::MONOCULAR) {

    this->declare_parameter("imageTopic", "robot" + to_string(agentId) + "/camera/image_color");
    string imageTopic = this->get_parameter("imageTopic").as_string();

    this->declare_parameter("reliableImageTransport", true);
    bool reliableImageTransport = this->get_parameter("reliableImageTransport").as_bool();

    this->declare_parameter("compressedImage", true);
    bool compressedImage = this->get_parameter("compressedImage").as_bool();

    image_subscriber_thread = std::thread([this, imageTopic, reliableImageTransport, compressedImage]() {
      auto sub_node = rclcpp::Node::make_shared("image_subscriber_thread_node");
      if (compressedImage) {
        compressed_image_subscriber = sub_node->create_subscription<sensor_msgs::msg::CompressedImage>(imageTopic,
          reliableImageTransport ? RELIABLE_QOS : BEST_EFFORT_QOS,
          std::bind(&OrbSlam3Mono::grab_compressed_image, this, std::placeholders::_1));
      }
      else {
        image_subscriber = sub_node->create_subscription<sensor_msgs::msg::Image>(imageTopic,
          reliableImageTransport ? RELIABLE_QOS : BEST_EFFORT_QOS,
          std::bind(&OrbSlam3Mono::grab_image, this, std::placeholders::_1));
      }
      rclcpp::spin(sub_node);
    });

    run();
  };

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_image_subscriber;
  rclcpp::TimerBase::SharedPtr timer_;
  std::thread image_subscriber_thread;

  void grab_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // image must be copied since it uses the conversion_mat_ for storage
      // which is asynchronously overwritten in the next callback invocation
      cv::Mat image_cpy = cv_ptr->image.clone();

      rclcpp::Time timestamp = msg->header.stamp;
      process_image(image_cpy, timestamp);

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
  }

  void grab_compressed_image(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // image must be copied since it uses the conversion_mat_ for storage
      // which is asynchronously overwritten in the next callback invocation
      cv::Mat image_cpy = cv_ptr->image.clone();

      rclcpp::Time timestamp = msg->header.stamp;
      process_image(image_cpy, timestamp);

    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->format.c_str());
    }
  }

  void process_image(const cv::Mat& image, const rclcpp::Time& timestamp) {
    double seconds = timestamp.nanoseconds() * 1.e-9;
    RCLCPP_INFO(this->get_logger(), "New Image. Timestamp: %f", seconds);
    Sophus::SE3f Tcw = pSLAM->TrackMonocular(image, seconds);

    publishRosVizTopics->publish_camera_pose(Tcw.inverse(), timestamp);

    newFrameProcessed = true;
    lastFrameTimestamp = timestamp;
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<OrbSlam3Mono>();

  rclcpp::shutdown();
  return 0;
}