#include <zed_cpu.hpp>

#include <memory>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>

namespace zed_cpu
{

ZedCameraNode::ZedCameraNode(
  const std::shared_ptr<ros::NodeHandle> & nh,
  const std::shared_ptr<image_transport::ImageTransport> & it)
: nh_(nh), it_(it)
{
  // ROS initialization
  node_name_ = ros::this_node::getName();
  left_image_pub_ = it_->advertise("rgb/left_image_raw", 1);
  right_image_pub_ = it_->advertise("rgb/right_image_raw", 1);
  left_image_compressed_pub_ = nh_->advertise<sensor_msgs::CompressedImage>("rgb/left_image/compressed", 1);
  right_image_compressed_pub_ = nh_->advertise<sensor_msgs::CompressedImage>("rgb/right_image/compressed", 1);
  imu_pub_ = nh_->advertise<sensor_msgs::Imu>("imu_data", 10);

  CameraInit();
  SensorInit();

  ROS_INFO("[%s] Node started", node_name_.c_str());
}

void ZedCameraNode::runCamera()
{
  PublishImages();
  // ros::spinOnce();
}

void ZedCameraNode::runIMU()
{
  while(1){
    PublishIMU();
  }

}

void ZedCameraNode::run()
{
  PublishImages();
  // PublishIMU();
}

void ZedCameraNode::CameraInit()
{
  // Initialize ZED camera
  sl_oc::video::VideoParams params;
  params.res = sl_oc::video::RESOLUTION::VGA;
  params.fps = sl_oc::video::FPS::FPS_30;
  params.verbose = sl_oc::VERBOSITY::INFO;

  // Create Video Capture
  cap_ = std::make_unique<sl_oc::video::VideoCapture>(params);
  if (!cap_->initializeVideo()) {
    ROS_ERROR("[%s] Cannot open camera video capture", node_name_.c_str());
    ros::shutdown();
    return;
  }

  ROS_INFO("[%s] Connected to camera sn: %d [%s]", node_name_.c_str(), cap_->getSerialNumber(), cap_->getDeviceName().c_str());
}

void ZedCameraNode::SensorInit()
{
  sens_ = std::make_unique<sl_oc::sensors::SensorCapture>(sl_oc::VERBOSITY::ERROR);

  std::vector<int> devs = sens_->getDeviceList();

  if (devs.size() == 0) {
    ROS_ERROR("[%s] No available ZED 2, ZED 2i or ZED Mini cameras", node_name_.c_str());
    ros::shutdown();
    return;
  }

  uint16_t fw_maior;
  uint16_t fw_minor;
  sens_->getFirmwareVersion(fw_maior, fw_minor);
  ROS_INFO("[%s] Connected to IMU firmware version: %d.%d", node_name_.c_str(), fw_maior, fw_minor);

  // Initialize the sensors
  if (!sens_->initializeSensors(devs[0])) {
    ROS_ERROR("[%s] IMU initialize failed", node_name_.c_str());
    ros::shutdown();
    return;
  }
}

void ZedCameraNode::PublishImages()
{
  // Get last available frame
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
    std_msgs::Header head;
    head.stamp = ros::Time::now();
    sensor_msgs::ImagePtr left_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", left_img).toImageMsg();
    sensor_msgs::ImagePtr right_msg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", right_img).toImageMsg();

    left_msg->header = head;
    right_msg->header = head;

    // Publish the left and right image messages
    // left_image_pub_.publish(left_msg);
    // right_image_pub_.publish(right_msg);



    // Create a CompressedImage message
    sensor_msgs::CompressedImage left_compressed_msg, right_compressed_msg;
    left_compressed_msg.header = left_msg->header; // Use the same header as the raw image
    left_compressed_msg.format = "jpeg";
    right_compressed_msg.header = right_msg->header; // Use the same header as the raw image
    right_compressed_msg.format = "jpeg";

    // Compress the image
    std::vector<uint8_t> left_buffer, right_buffer;
    std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90}; // Adjust quality (0-100)
    cv::imencode(".jpg", left_img, left_buffer, compression_params);
    cv::imencode(".jpg", right_img, right_buffer, compression_params);

    // Fill the CompressedImage message
    left_compressed_msg.data = left_buffer;
    right_compressed_msg.data = right_buffer;

    // Publish the compressed image
    left_image_compressed_pub_.publish(left_compressed_msg);
    right_image_compressed_pub_.publish(right_compressed_msg);
  }
}

void ZedCameraNode::PublishIMU()
{
  // Get IMU data with a timeout of 5 milliseconds
  const sl_oc::sensors::data::Imu imu_data = sens_->getLastIMUData(2000);

  if (imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
    // Create a sensor_msgs/Imu message
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_frame";

    // Convert the IMU data to the sensor_msgs/Imu message fields
    imu_msg.linear_acceleration.x = -imu_data.aX;
    imu_msg.linear_acceleration.y = imu_data.aY;
    imu_msg.linear_acceleration.z = imu_data.aZ;

    imu_msg.angular_velocity.x = -imu_data.gX;
    imu_msg.angular_velocity.y = imu_data.gY;
    imu_msg.angular_velocity.z = imu_data.gZ;

    // Publish the sensor_msgs/Imu message
    imu_pub_.publish(imu_msg);
    // ROS_INFO_STREAM("publish IMU");
  }
  else{
    ROS_DEBUG_STREAM("IMU data not valid");
  }
}

}  // namespace zed_cpu