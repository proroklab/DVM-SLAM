#include "System.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std;

class ImageGrabber : public rclcpp::Node {
public:
  ImageGrabber() : Node("image_grabber") {
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/robot/camera/image_color", 1,
        std::bind(&ImageGrabber::grab_image, this, std::placeholders::_1));

    pSLAM = new ORB_SLAM3::System(voc_file, settings_file,
                                  ORB_SLAM3::System::MONOCULAR, true);
  };

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  ORB_SLAM3::System *pSLAM;

  string voc_file =
      "/home/joshuabird/Desktop/Parallels\ Shared\ Folders/ubuntuSharedFolder/"
      "part_II_project/src/orb_slam3_ros/orb_slam3/Vocabulary/ORBvoc.txt";
  string settings_file = "/home/joshuabird/Desktop/Parallels\ Shared\ Folders/"
                         "ubuntuSharedFolder/part_II_project/src/webots_sim/"
                         "worlds/webots.yaml";

  void grab_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      // image must be copied since it uses the conversion_mat_ for storage
      // which is asynchronously overwritten in the next callback invocation
      cv::Mat image_cpy = cv_ptr->image.clone();

      rclcpp::Time timestamp = msg->header.stamp;
      double seconds = timestamp.nanoseconds() * 1.e-9;
      RCLCPP_INFO(this->get_logger(), "New Image. Timestamp: %f", seconds);
      Sophus::SE3f Tcw = pSLAM->TrackMonocular(image_cpy, seconds);

    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.",
                   msg->encoding.c_str());
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ImageGrabber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}