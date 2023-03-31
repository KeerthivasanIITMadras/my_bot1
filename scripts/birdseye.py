#!/usr/bin/env python
import rospy
import cv2
import numpy as np
import math
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from image_geometry import PinholeCameraModel
from std_msgs.msg import Header


class PinholeCamera:
    def __init__(self):
        self.sensor_info = rospy.wait_for_message(
            '/stereo_camera/right/camera_info', CameraInfo, timeout=5)
        self.image_subscriber = rospy.Subscriber(
            "/stereo_camera/right/image_raw", Image, self.callback)
        self.camera_angle = 0.45
        self.bridge = CvBridge()
        self.uv = None
        self.model = PinholeCameraModel()
        self.camera_height = 0.118
        # world coordinate z axis
        self.n = np.array([[0], [0], [1]])
        # pitch rotation matrix
        self.R = np.array([[1, 0, 0], [0, math.cos(self.camera_angle), math.sin(self.camera_angle)], [
            0, math.sin(self.camera_angle), math.cos(self.camera_angle)]])
        print(self.model.R)
        print(self.R)
        self.nc = np.transpose(self.R.dot(self.n))
        self.coordinates_wrt_camera = []

    def callback(self, img: Image):
        self.coordinates_wrt_camera = []
        image = self.bridge.imgmsg_to_cv2(img, 'bgr8')
        # Set the camera parameters from the :class:`sensor_msgs.msg.CameraInfo` message
        self.model.fromCameraInfo(self.sensor_info)
        # Applies the rectification specified by camera parameters :math:`K` and and :math:`D` to image `raw` and writes the resulting image `rectified`
        self.model.rectifyImage(image, image)
        img_grey = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        img_canny = cv2.Canny(img_grey, 0, 255)
        self.uv = list(zip(np.where(img_canny > 0)[
            0], np.where(img_canny > 0)[1]))
        self.d3point_camera_reference()

    def d3point_camera_reference(self):
        for point in self.uv:
            # Kinv(u,v,1),unit vector
            (x, y, z) = self.model.projectPixelTo3dRay(point)
            den = self.nc.dot([[x], [y], [z]])
            num = self.camera_height*np.array([[x], [y], [z]])
            # print(f"Numerator = {num} and Denominator = {den}")
            [xc, yc, zc] = num/den
            # print(xc, yc, zc)
            self.coordinates_wrt_camera.append([xc, yc, zc])


if __name__ == '__main__':
    rospy.init_node('birdseye', anonymous=True)
    obj = PinholeCamera()
    pcl_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
    rospy.spin()
    # while not rospy.is_shutdown():
    #     header = Header()
    #     header.stamp = rospy.Time.now()
    #     header.frame_id = '/odom'
    #     scaled_polygon_pcl = pcl2.create_cloud_xyz32(
    #         header, obj.coordinates_wrt_camera)
    #     pcl_pub.publish(scaled_polygon_pcl)
    #     rate.sleep()
