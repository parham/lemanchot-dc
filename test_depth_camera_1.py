#!/usr/bin/env python

import phm
import rospy
import logging
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import message_filters

import dynamic_reconfigure.client as dclient

def callback_visible(data):
    try:
        img = np.frombuffer(data.data, dtype=np.uint8).reshape(
            data.height, data.width, -1)
    except Exception as ex:
        print(ex)

    print('viz')
    cv2.imshow("Visible Window", img)
    cv2.waitKey(10)

def callback_depth(data):
    try:
        img = np.frombuffer(data.data, dtype=np.uint16).reshape(
            data.height, data.width, -1)
    except Exception as ex:
        print(ex)

    print('depth')
    cv2.imshow("Depth Window", img)
    cv2.waitKey(10)

def callback_thermal(data):
    try:
        img = np.frombuffer(data.data, dtype=np.uint8).reshape(
            data.height, data.width, -1)
    except Exception as ex:
        print(ex)

    cv2.imshow("Depth Window", img)
    cv2.waitKey(10)

def shutdown():
    rospy.loginfo('Node is shutting down ...')
    cv2.destroyAllWindows()

def callback(config):
    # rospy.loginfo("Config set to {int_param}, {double_param}, {str_param}, {bool_param}, {size}".format(**config))
    print(config)

def main(args):
    # print('Node Test is being started ...')
    logging.info('Node Test is being started ...')
    rospy.init_node('image_converter', anonymous=True)
    rospy.on_shutdown(shutdown)
    dclient.Client('/phm/phm_flir_spinnaker', timeout=30, config_callback=callback)

    thermal_sub = rospy.Subscriber(
            "/phm/thermal_camera/image", Image, callback_thermal, queue_size=10)
    # depth_sub = rospy.Subscriber(
    #         "/phm/depth_camera/aligned_depth_to_color/image_raw", Image, callback_depth, queue_size=10)
    # vis_sub = rospy.Subscriber(
    #         "/phm/depth_camera/color/image_raw", Image, callback_visible, queue_size=10)


    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
