#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge

class StereoRectifier:
    def __init__(self):
        rospy.init_node("stereo_rectify_node", anonymous=True)

        # Load camera parameters
        self.load_camera_params()

        # Create CvBridge
        self.bridge = CvBridge()

        # Subscribe to compressed image topics
        rospy.Subscriber("/zed_node/rgb/left_image/compressed", CompressedImage, self.left_image_callback, queue_size=1)
        rospy.Subscriber("/zed_node/rgb/right_image/compressed", CompressedImage, self.right_image_callback, queue_size=1)

        # Publish rectified compressed images
        self.left_pub = rospy.Publisher("/zed_node/rgb/left_image_rect/compressed", CompressedImage, queue_size=1)
        self.right_pub = rospy.Publisher("/zed_node/rgb/right_image_rect/compressed", CompressedImage, queue_size=1)

        rospy.loginfo("Stereo Rectifier Node Started")

    def load_camera_params(self):
        """ Load camera intrinsics and compute rectification maps """

        # Set numpy print options
        np.set_printoptions(suppress=True, precision=6)

        # === Replaced Camera Intrinsics ===
        # self.K_left = np.array([[720.6829, 0, 626.9098],
        #                         [0, 722.6085, 385.3141],
        #                         [0, 0, 1]], dtype=np.float64)
        #
        # self.D_left = np.array([0.1810, 0.5509, 0.0103, -0.0219, 0.1745], dtype=np.float64)
        #
        # self.K_right = np.array([[725.7787, 0, 641.5171],
        #                          [0, 729.5106, 360.3579],
        #                          [0, 0, 1]], dtype=np.float64)
        #
        # self.D_right = np.array([0.1465, 0.8906, -0.0015, -0.0053, -0.2765], dtype=np.float64)
        #
        # # === Replaced Stereo Extrinsics ===
        # R_stereo = np.array([[0.9997,  0.0044, -0.0245],
        #                      [-0.0039, 0.9997,  0.0222],
        #                      [0.0246, -0.0221,  0.9995]], dtype=np.float64)
        #
        # # MATLAB unit is mm → convert to meters
        # T_stereo = np.array([[0.1171463], [-0.0003619], [0.0058835]], dtype=np.float64)

        self.K_left = np.array([[725.7795, 0, 641.5163],
                                [0, 729.5113, 360.3576],
                                [0, 0, 1]], dtype=np.float64)

        self.D_left = np.array([0.1465, 0.8906, -0.0015, -0.0053, -0.2765], dtype=np.float64)

        self.K_right = np.array([[720.6837, 0, 626.9076],
                                 [0, 722.6091, 385.3124],
                                 [0, 0, 1]], dtype=np.float64)

        self.D_right = np.array([0.1810, 0.5509, 0.0103, -0.0219, 0.1745], dtype=np.float64)

        # ==== Updated Extrinsic Parameters ====

        R_stereo = np.array([[ 0.9997, -0.0039,  0.0246],
                             [ 0.0044,  0.9997, -0.0221],
                             [-0.0245,  0.0222,  0.9995]], dtype=np.float64)

        # Convert from mm to meters
        T_stereo = np.array([[-0.1172560], [-0.0000246], [-0.0030037]], dtype=np.float64)


        # Image size (update if needed)
        self.image_size = (1280, 720)

        # Compute rectification matrices
        R_left, R_right, P_left, P_right, _, _, _ = cv2.stereoRectify(
            self.K_left, self.D_left, self.K_right, self.D_right,
            self.image_size, R_stereo, T_stereo, alpha=0
        )

        # Compute rectification maps
        self.map_left_x, self.map_left_y = cv2.initUndistortRectifyMap(
            self.K_left, self.D_left, R_left, P_left, self.image_size, cv2.CV_32FC1
        )

        self.map_right_x, self.map_right_y = cv2.initUndistortRectifyMap(
            self.K_right, self.D_right, R_right, P_right, self.image_size, cv2.CV_32FC1
        )

        # Print for verification
        rospy.loginfo("Left Rectification Matrix:\n%s", R_left)
        rospy.loginfo("Right Rectification Matrix:\n%s", R_right)
        rospy.loginfo("Left Projection Matrix:\n%s", P_left)
        rospy.loginfo("Right Projection Matrix:\n%s", P_right)
        rospy.loginfo("Camera parameters loaded and rectification maps computed.")

    def rectify_image(self, img, map_x, map_y):
        """ Rectify image using precomputed rectification maps """
        return cv2.remap(img, map_x, map_y, interpolation=cv2.INTER_LINEAR)

    def encode_compressed_image(self, img):
        """ Convert rectified image to CompressedImage format """
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]).tobytes()
        return msg

    def left_image_callback(self, msg):
        """ Callback for left compressed image """
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        img_rect = self.rectify_image(img, self.map_left_x, self.map_left_y)

        img_msg = self.encode_compressed_image(img_rect)
        img_msg.header = msg.header
        self.left_pub.publish(img_msg)

    def right_image_callback(self, msg):
        """ Callback for right compressed image """
        np_arr = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        img_rect = self.rectify_image(img, self.map_right_x, self.map_right_y)

        img_msg = self.encode_compressed_image(img_rect)
        img_msg.header = msg.header
        self.right_pub.publish(img_msg)

if __name__ == "__main__":
    try:
        node = StereoRectifier()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
