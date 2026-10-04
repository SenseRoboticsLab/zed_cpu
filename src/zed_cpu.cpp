#include <zed_cpu.hpp>

#include <future>
#include <memory>
#include <vector>

#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <sensor_msgs/Imu.h>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>

namespace zed_cpu
{

namespace
{
// Accelerometer calibration results (per-axis scale and temperature-corrected bias),
// applied to the raw sensor-frame readings as: corrected = (raw - bias) / scale
constexpr double kAccelScale[3] = {1.00001, 0.996776, 0.997335};
constexpr double kAccelBias[3] = {-0.112271, 0.0700745, 0.0688659};
}  // namespace

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
  ros::Rate rate(200);
  while(1){
    PublishIMU();
    rate.sleep();
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
  params.res = sl_oc::video::RESOLUTION::HD720;
  params.fps = sl_oc::video::FPS::FPS_15;
  params.verbose = sl_oc::VERBOSITY::INFO;

  // //camera parameter
  // double fx = 355.3514, fy = 355.9311, cx = 337.8985, cy = 194.2103;
  // double k1 = 0.3466, k2=0.1725, p1 = 0.0150, p2 = 0.0063;
  // camera_matrix_ = (cv::Mat_<double>(3,3)<<fx, 0, cx, 0, fy, cy, 0, 0, 1);
  // dist_coeffs_ = (cv::Mat_<double>(1,4)<<k1, k2, p1, p2);
  //

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
  // Wait for the next frame (returns the previous one again if the wait times out)
  const sl_oc::video::Frame frame = cap_->getLastFrame();

  // Skip timeouts and frames that were already published
  if (frame.data == nullptr || frame.frame_id == last_frame_id_) {
    return;
  }
  last_frame_id_ = frame.frame_id;

  std_msgs::Header head;
  head.stamp = ros::Time::now();

  // The frame is the left and right images side by side in YUYV format
  const cv::Mat frame_yuv(frame.height, frame.width, CV_8UC2, frame.data);
  const int eye_width = frame.width / 2;
  const std::vector<int> jpeg_params = {cv::IMWRITE_JPEG_QUALITY, 90};

  // Convert one half of the frame to BGR and JPEG-encode it straight into the message.
  // The buffers are members so their memory is reused from frame to frame.
  auto encode_eye = [&](int x_offset, cv::Mat & bgr, sensor_msgs::CompressedImage & msg) {
    cv::cvtColor(
      frame_yuv(cv::Rect(x_offset, 0, eye_width, frame.height)), bgr, cv::COLOR_YUV2BGR_YUYV);
    msg.header = head;
    msg.format = "jpeg";
    cv::imencode(".jpg", bgr, msg.data, jpeg_params);
  };

  // Encode the right image on another core while this thread does the left one
  auto right_done = std::async(
    std::launch::async, encode_eye, eye_width, std::ref(right_bgr_), std::ref(right_msg_));
  encode_eye(0, left_bgr_, left_msg_);
  right_done.get();

  left_image_compressed_pub_.publish(left_msg_);
  right_image_compressed_pub_.publish(right_msg_);
}

void ZedCameraNode::PublishIMU()
{
  // Get IMU data with a timeout of 5 milliseconds
  const sl_oc::sensors::data::Imu imu_data = sens_->getLastIMUData(1);

  if (imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
    // Create a sensor_msgs/Imu message
    sensor_msgs::Imu imu_msg;
    imu_msg.header.stamp = ros::Time::now();
    imu_msg.header.frame_id = "imu_frame";

    // Apply accelerometer calibration (scale + temperature-corrected bias) on the
    // raw sensor-frame axes before converting to the ROS axis convention
    const double accel_x = (imu_data.aX - kAccelBias[0]) / kAccelScale[0];
    const double accel_y = (imu_data.aY - kAccelBias[1]) / kAccelScale[1];
    const double accel_z = (imu_data.aZ - kAccelBias[2]) / kAccelScale[2];

    // Convert the IMU data to the sensor_msgs/Imu message fields
    imu_msg.linear_acceleration.x = -accel_x;
    imu_msg.linear_acceleration.y = accel_y;
    imu_msg.linear_acceleration.z = -accel_z;

    imu_msg.angular_velocity.x = -imu_data.gX;
    imu_msg.angular_velocity.y = imu_data.gY;
    imu_msg.angular_velocity.z = -imu_data.gZ;

    // Publish the sensor_msgs/Imu message
    imu_pub_.publish(imu_msg);
    // ROS_INFO_STREAM("publish IMU");
  }
  else{
    ROS_DEBUG_STREAM("IMU data not valid");
  }
}

}  // namespace zed_cpu