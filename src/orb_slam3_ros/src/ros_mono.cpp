#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std;

class ImageGrabber : public rclcpp::Node {
public:
  ImageGrabber() : Node("image_grabber") {
    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/Mavic_3_PRO/camera/image_color", 1,
        bind(&ImageGrabber::grab_image, this, placeholders::_1));
  };

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;

  void grab_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
      cv_bridge::CvImagePtr cv_ptr =
          cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // Do something with the image (cv_ptr->image)
      cv::imshow("Image Viewer", cv_ptr->image);
      cv::waitKey(1);
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