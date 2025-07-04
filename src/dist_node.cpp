#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <compressed_image_transport/compressed_publisher.h>

class ImageUndistorter
{
public:
    ImageUndistorter(ros::NodeHandle &nh)
        : it_(nh)
    {
        // Parameters
        nh.param("camera_matrix/fx", fx_, 0.0);
        nh.param("camera_matrix/fy", fy_, 0.0);
        nh.param("camera_matrix/cx", cx_, 0.0);
        nh.param("camera_matrix/cy", cy_, 0.0);
        nh.param("distortion_coefficients/k1", k1_, 0.0);
        nh.param("distortion_coefficients/k2", k2_, 0.0);
        nh.param("distortion_coefficients/p1", p1_, 0.0);
        nh.param("distortion_coefficients/p2", p2_, 0.0);

        //print parameters
        ROS_INFO("camera_matrix/fx: %f", fx_);
        ROS_INFO("camera_matrix/fy: %f", fy_);
        ROS_INFO("camera_matrix/cx: %f", cx_);
        ROS_INFO("camera_matrix/cy: %f", cy_);
        ROS_INFO("distortion_coefficients/k1: %f", k1_);
        ROS_INFO("distortion_coefficients/k2: %f", k2_);
        ROS_INFO("distortion_coefficients/p1: %f", p1_);
        ROS_INFO("distortion_coefficients/p2: %f", p2_);

        // Initialize camera matrix and distortion coefficients
        camera_matrix_ = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
        dist_coeffs_ = (cv::Mat_<double>(1, 4) << k1_, k2_, p1_, p2_);

        // Subscribers and Publishers
        left_image_sub_ = nh.subscribe("/zed_node/rgb/left_image/compressed", 1, &ImageUndistorter::leftImageCallback,
                                       this);
        right_image_sub_ = nh.subscribe("/zed_node/rgb/right_image/compressed", 1,
                                        &ImageUndistorter::rightImageCallback, this);

        left_image_pub_ = nh.advertise<sensor_msgs::CompressedImage>("/zed_node/rgb/left_image/undistorted/compressed",
                                                                     1);
        right_image_pub_ = nh.advertise<sensor_msgs::CompressedImage>(
            "/zed_node/rgb/right_image/undistorted/compressed", 1);
    }

private:
    void processAndPublish(const sensor_msgs::CompressedImageConstPtr &msg, const ros::Publisher &pub)
    {
        try {
            // Convert compressed image to OpenCV format using cv_bridge
            cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
            cv_ptr->header = msg->header;
            cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
            cv_ptr->image = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);

            // Undistort the image
            cv::Mat undistorted;
            cv::undistort(cv_ptr->image, undistorted, camera_matrix_, dist_coeffs_);

            // Crop the undistorted image to remove black borders
            cv::Rect valid_roi;
            cv::Mat new_camera_matrix = cv::getOptimalNewCameraMatrix(camera_matrix_, dist_coeffs_,
                                                                      cv_ptr->image.size(), 1, cv_ptr->image.size(),
                                                                      &valid_roi);
            cv::Mat cropped = undistorted(valid_roi);

            // Update camera matrix to reflect the new ROI
            new_camera_matrix.at<double>(0, 2) -= valid_roi.x; // Adjust cx
            new_camera_matrix.at<double>(1, 2) -= valid_roi.y; // Adjust cy

            // Convert to compressed image and publish
            std_msgs::Header header = msg->header;
            std::vector<uchar> compressed_buffer;
            cv::imencode(".jpg", cropped, compressed_buffer);

            sensor_msgs::CompressedImage compressed_msg;
            compressed_msg.header = header;
            compressed_msg.format = "jpeg";
            compressed_msg.data = compressed_buffer;

            pub.publish(compressed_msg);
        } catch (const cv::Exception &e) {
            ROS_ERROR("OpenCV exception: %s", e.what());
        }
    }

    void leftImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
    {
        processAndPublish(msg, left_image_pub_);
    }

    void rightImageCallback(const sensor_msgs::CompressedImageConstPtr &msg)
    {
        processAndPublish(msg, right_image_pub_);
    }

    image_transport::ImageTransport it_;
    ros::Subscriber left_image_sub_;
    ros::Subscriber right_image_sub_;
    ros::Publisher left_image_pub_;
    ros::Publisher right_image_pub_;

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    double fx_, fy_, cx_, cy_;
    double k1_, k2_, p1_, p2_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_undistorter");
    ros::NodeHandle nh;

    ImageUndistorter undistorter(nh);

    ros::spin();
    return 0;
}
