#include "System.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/highgui/highgui.hpp"
#include "orb_slam3_wrapper.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/detail/imu__struct.hpp>

using namespace std;

class OrbSlam3ImuMono : public OrbSlam3Wrapper {
public:
  OrbSlam3ImuMono(string voc_file, string settings_file)
    : OrbSlam3Wrapper("orb_slam3_imu_mono", voc_file, settings_file, ORB_SLAM3::System::IMU_MONOCULAR)
    , sync_thread(&OrbSlam3ImuMono::sync_with_imu, this) {
    image_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
      "/robot1/camera/image_color", 5, std::bind(&OrbSlam3ImuMono::grab_image, this, std::placeholders::_1));
    imu_subscriber = this->create_subscription<sensor_msgs::msg::Imu>(
      "/robot1/imu", 1000, std::bind(&OrbSlam3ImuMono::grab_imu, this, std::placeholders::_1));
  };

private:
  std::thread sync_thread;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscriber;

  queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
  queue<sensor_msgs::msg::Image::SharedPtr> imageBuf;
  std::mutex imuBufMutex;
  std::mutex imageBufMutex;

  void grab_image(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "img: %f", static_cast<rclcpp::Time>(msg->header.stamp).seconds());
    imageBufMutex.lock();
    // Max buffer size
    if (imageBuf.size() >= 5)
      imageBuf.pop();
    imageBuf.push(msg);
    imageBufMutex.unlock();
  }

  void grab_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "imu: %f", static_cast<rclcpp::Time>(msg->header.stamp).seconds());
    imuBufMutex.lock();
    imuBuf.push(msg);
    imuBufMutex.unlock();
  }

  cv::Mat get_image(const sensor_msgs::msg::Image::ConstSharedPtr img_msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
      return cv_ptr->image.clone();
    }
    else {
      std::cout << "Error type" << std::endl;
      return cv_ptr->image.clone();
    }
  }

  void sync_with_imu() {
    while (1) {
      if (!imageBuf.empty() && !imuBuf.empty()) {
        cv::Mat im;
        double tIm = 0;

        tIm = static_cast<rclcpp::Time>(imageBuf.front()->header.stamp).seconds();
        // If the oldest image is newer than the latest imu data
        if (tIm > static_cast<rclcpp::Time>(imuBuf.back()->header.stamp).seconds())
          continue;

        // RCLCPP_INFO(this->get_logger(), "tIm: %f", tIm);

        this->imageBufMutex.lock();
        im = get_image(imageBuf.front());
        rclcpp::Time msg_time = imageBuf.front()->header.stamp;
        imageBuf.pop();
        this->imageBufMutex.unlock();

        vector<ORB_SLAM3::IMU::Point> vImuMeas;
        Eigen::Vector3f Wbb;
        imageBufMutex.lock();
        if (!imuBuf.empty()) {
          // Load imu measurements from buffer
          vImuMeas.clear();
          //  While the oldest imu sample is younger than the current image
          // RCLCPP_INFO(this->get_logger(), "imuBuf.empty(): %d",
          // imuBuf.empty()); RCLCPP_INFO(this->get_logger(), "oldest imu
          // sample: %f",
          //             static_cast<rclcpp::Time>(imuBuf.front()->header.stamp)
          //                 .seconds());
          while (!imuBuf.empty() && static_cast<rclcpp::Time>(imuBuf.front()->header.stamp).seconds() <= tIm) {
            // RCLCPP_INFO(this->get_logger(), "aaaa");
            double t = static_cast<rclcpp::Time>(imuBuf.front()->header.stamp).seconds();

            cv::Point3f acc(imuBuf.front()->linear_acceleration.x, imuBuf.front()->linear_acceleration.y,
              imuBuf.front()->linear_acceleration.z);

            cv::Point3f gyr(imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y,
              imuBuf.front()->angular_velocity.z);

            vImuMeas.push_back(ORB_SLAM3::IMU::Point(acc, gyr, t));

            Wbb << imuBuf.front()->angular_velocity.x, imuBuf.front()->angular_velocity.y,
              imuBuf.front()->angular_velocity.z;

            imuBuf.pop();
          }
        }
        imageBufMutex.unlock();
        // RCLCPP_INFO(this->get_logger(), "vImuMeas size: %d",
        // vImuMeas.size());

        // ORB-SLAM3 runs in TrackMonocular()
        Sophus::SE3f Tcw = pSLAM->TrackMonocular(im, tIm, vImuMeas);

        // publish_topics(msg_time, Wbb);
      }

      std::chrono::milliseconds tSleep(1);
      std::this_thread::sleep_for(tSleep);
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
  auto node = std::make_shared<OrbSlam3ImuMono>(voc_file, settings_file);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}