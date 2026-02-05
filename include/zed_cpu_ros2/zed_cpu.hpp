#ifndef ZED_CPU_ROS2__ZED_CPU_HPP_
#define ZED_CPU_ROS2__ZED_CPU_HPP_

#include <memory>
#include <thread>
#include <atomic>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <opencv2/opencv.hpp>

#include "zed_lib/sensorcapture.hpp"
#include "zed_lib/videocapture.hpp"

namespace zed_cpu_ros2
{

class ZedCameraNode : public rclcpp::Node
{
public:
  explicit ZedCameraNode(const rclcpp::NodeOptions & options);
  virtual ~ZedCameraNode();

private:
  void CameraInit();
  void SensorInit();
  void PublishImages();
  void PublishIMU();
  void RunCameraLoop();
  void RunIMULoop();

  // Communication
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  image_transport::Publisher left_image_pub_;
  image_transport::Publisher right_image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr left_image_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr right_image_compressed_pub_;

  // Tools
  std::shared_ptr<image_transport::ImageTransport> it_;

  // ZED Hardware
  std::unique_ptr<sl_oc::video::VideoCapture> cap_;
  std::unique_ptr<sl_oc::sensors::SensorCapture> sens_;
  
  // Internal logic
  cv::Mat camera_matrix_, dist_coeffs_;
  std::thread camera_thread_;
  std::thread imu_thread_;
  std::atomic<bool> running_;
};

}  // namespace zed_cpu_ros2

#endif  // ZED_CPU_ROS2__ZED_CPU_HPP_
