#include "zed_cpu_ros2/zed_cpu.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  auto node = std::make_shared<zed_cpu_ros2::ZedCameraNode>(rclcpp::NodeOptions());
  
  rclcpp::spin(node);

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
