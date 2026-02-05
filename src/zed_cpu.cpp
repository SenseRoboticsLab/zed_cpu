#include "zed_cpu_ros2/zed_cpu.hpp"

#include <chrono>
#include <cv_bridge/cv_bridge.h>

using namespace std::chrono_literals;

namespace zed_cpu_ros2
{

ZedCameraNode::ZedCameraNode(const rclcpp::NodeOptions & options)
: Node("zed_camera_node", options), running_(false)
{
  // ROS initialization
  // it_ = std::make_shared<image_transport::ImageTransport>(shared_from_this()); 
  // NOTE: shared_from_this() cannot be called in constructor. 
  // We will initialize image_transport and publishers in a slightly different way or use `image_transport::create_publisher`.
  
  // Actually, we can't use shared_from_this() in constructor.
  // We can treat `it_` initialization lates or use the static helper if available.
  // But common pattern is to just use correct helper functions.

  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("~/imu_data", 10);
  
  left_image_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("~/rgb/left_image/compressed", 1);
  right_image_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("~/rgb/right_image/compressed", 1);

  // We delay IT initialization/publishing to when we have shared_ptr or use `create_publisher` which takes `node_interfaces`.
  // Ideally, use `image_transport::create_publisher(this, ...)` directly if supported.
  // As of generic ROS 2, `image_transport::create_publisher` expects `rclcpp::Node*` or `rclcpp::Node::SharedPtr`.
  // Pass `this` works for `rclcpp::Node*` overloads.
  
  // left_image_pub_ = image_transport::create_publisher(this, "rgb/left_image_raw");
  // right_image_pub_ = image_transport::create_publisher(this, "rgb/right_image_raw");

  CameraInit();
  SensorInit();

  running_ = true;
  camera_thread_ = std::thread(&ZedCameraNode::RunCameraLoop, this);
  imu_thread_ = std::thread(&ZedCameraNode::RunIMULoop, this);

  RCLCPP_INFO(this->get_logger(), "Node started");
}

ZedCameraNode::~ZedCameraNode()
{
  running_ = false;
  if (camera_thread_.joinable()) camera_thread_.join();
  if (imu_thread_.joinable()) imu_thread_.join();
}

void ZedCameraNode::RunCameraLoop()
{
  while(running_ && rclcpp::ok()) {
    PublishImages();
    // Yield to avoid 100% CPU if capture is non-blocking, but getLastFrame usually blocks or sleeps.
    // If getLastFrame is non-blocking and returns empty, we should sleep.
    // Assuming blocking for now based on original code usage.
  }
}

void ZedCameraNode::RunIMULoop()
{
  rclcpp::Rate rate(200);
  while(running_ && rclcpp::ok()){
    PublishIMU();
    rate.sleep();
  }
}

void ZedCameraNode::CameraInit()
{
  // Initialize ZED camera
  sl_oc::video::VideoParams params;
  params.res = sl_oc::video::RESOLUTION::HD720;
  params.fps = sl_oc::video::FPS::FPS_15;
  params.verbose = sl_oc::VERBOSITY::INFO;

  // Create Video Capture
  cap_ = std::make_unique<sl_oc::video::VideoCapture>(params);
  if (!cap_->initializeVideo()) {
    RCLCPP_ERROR(this->get_logger(), "Cannot open camera video capture");
    // In ROS 2, we can't easily shut down the whole system from a constructor/init, 
    // but we can exit the process or just return.
    // ros::shutdown() equivalent:
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Connected to camera sn: %d [%s]", 
    cap_->getSerialNumber(), cap_->getDeviceName().c_str());
}

void ZedCameraNode::SensorInit()
{
  sens_ = std::make_unique<sl_oc::sensors::SensorCapture>(sl_oc::VERBOSITY::ERROR);

  std::vector<int> devs = sens_->getDeviceList();

  if (devs.size() == 0) {
    RCLCPP_ERROR(this->get_logger(), "No available ZED 2, ZED 2i or ZED Mini cameras");
    rclcpp::shutdown();
    return;
  }

  uint16_t fw_maior;
  uint16_t fw_minor;
  sens_->getFirmwareVersion(fw_maior, fw_minor);
  RCLCPP_INFO(this->get_logger(), "Connected to IMU firmware version: %d.%d", fw_maior, fw_minor);

  // Initialize the sensors
  if (!sens_->initializeSensors(devs[0])) {
    RCLCPP_ERROR(this->get_logger(), "IMU initialize failed");
    rclcpp::shutdown();
    return;
  }
}

void ZedCameraNode::PublishImages()
{
  // Get last available frame
  if (!cap_) return;
  const sl_oc::video::Frame frame = cap_->getLastFrame();

  // Process and publish the frame
  if (frame.data != nullptr) {
    cv::Mat frame_yuv = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
    cv::Mat frame_bgr;
    cv::cvtColor(frame_yuv, frame_bgr, cv::COLOR_YUV2BGR_YUYV);

    // Split the frame into left and right images
    cv::Mat left_img = frame_bgr(cv::Rect(0, 0, frame_bgr.cols / 2, frame_bgr.rows));
    cv::Mat right_img = frame_bgr(cv::Rect(frame_bgr.cols / 2, 0, frame_bgr.cols / 2, frame_bgr.rows));

    // Convert the OpenCV images to ROS image messages
    // Use cv_bridge::CvImage(...).toImageMsg()
    // In ROS 2, toImageMsg produces sensor_msgs::msg::Image::SharedPtr or similar.

    std_msgs::msg::Header head;
    head.stamp = this->now();
    head.frame_id = "zed_camera_link"; // Good practice to have a frame_id

    sensor_msgs::msg::Image::SharedPtr left_msg =
      cv_bridge::CvImage(head, "bgr8", left_img).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr right_msg =
      cv_bridge::CvImage(head, "bgr8", right_img).toImageMsg();

    // Publish the left and right image messages
    // left_image_pub_.publish(left_msg);
    // right_image_pub_.publish(right_msg);

    // Create a CompressedImage message
    sensor_msgs::msg::CompressedImage left_compressed_msg, right_compressed_msg;
    left_compressed_msg.header = left_msg->header; 
    left_compressed_msg.format = "jpeg";
    right_compressed_msg.header = right_msg->header;
    right_compressed_msg.format = "jpeg";

    // Compress the image
    std::vector<uint8_t> left_buffer, right_buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90}; 
    cv::imencode(".jpg", left_img, left_buffer, compression_params);
    cv::imencode(".jpg", right_img, right_buffer, compression_params);

    // Fill the CompressedImage message
    left_compressed_msg.data = left_buffer;
    right_compressed_msg.data = right_buffer;

    // Publish the compressed image
    left_image_compressed_pub_->publish(left_compressed_msg);
    right_image_compressed_pub_->publish(right_compressed_msg);
  }
}

void ZedCameraNode::PublishIMU()
{
  if (!sens_) return;
  // Get IMU data with a timeout of 5 milliseconds
  const sl_oc::sensors::data::Imu imu_data = sens_->getLastIMUData(1);

  if (imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
    // Create a sensor_msgs/Imu message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_frame";

    // Convert the IMU data
    // Coordinate system transform might be needed depending on ROS standards (ENU vs NED).
    // Original code: x = -aX, y = aY, z = -aZ. Keeping as is.
    imu_msg.linear_acceleration.x = -imu_data.aX;
    imu_msg.linear_acceleration.y = imu_data.aY;
    imu_msg.linear_acceleration.z = -imu_data.aZ;

    imu_msg.angular_velocity.x = -imu_data.gX;
    imu_msg.angular_velocity.y = imu_data.gY;
    imu_msg.angular_velocity.z = -imu_data.gZ;

    imu_pub_->publish(imu_msg);
  }
  else{
    // RCLCPP_DEBUG(this->get_logger(), "IMU data not valid");
  }
}

}  // namespace zed_cpu_ros2
