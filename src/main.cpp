#include "zed_cpu.hpp"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "zed_camera");

  auto nh = std::make_shared<ros::NodeHandle>("~");
  auto it = std::make_shared<image_transport::ImageTransport>(*nh);

  zed_cpu::ZedCameraNode zed_camera_node(nh, it);
  std::thread imu_thread(&zed_cpu::ZedCameraNode::runIMU, &zed_camera_node);
  // std::thread camera_thread(&zed_cpu::ZedCameraNode::runCamera, &zed_camera_node);
  while (ros::ok()) {
    zed_camera_node.run();
    ros::spinOnce();
  }

  ros::shutdown();

  return EXIT_SUCCESS;
}