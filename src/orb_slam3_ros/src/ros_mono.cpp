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
        "/Mavic_2_PRO/camera/image_color", 1,
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
                         "ubuntuSharedFolder/part_II_project/src/orb_slam3_ros/"
                         "orb_slam3/Examples/Monocular/EuRoC.yaml";

  void grab_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

      rclcpp::Time timestamp = msg->header.stamp;
      double seconds = timestamp.nanoseconds() * 1.e-9;
      Sophus::SE3f Tcw = pSLAM->TrackMonocular(cv_ptr->image, seconds);

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