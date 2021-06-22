#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image, CameraInfo
import message_filters

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

def callback_sync(depth, viz):
    try:
        vimg = np.frombuffer(viz.data, dtype=np.uint8).reshape(viz.height, viz.width, -1)
        dimg = np.frombuffer(depth.data, dtype=np.uint16).reshape(depth.height, depth.width, -1)
    except Exception as ex:
        print(ex)

    cv2.imshow("Visible Window", vimg)
    cv2.imshow("Depth Window", dimg)
    cv2.waitKey(10)


def main(args):

    rospy.init_node('image_converter', anonymous=True)

    # depth_sub = rospy.Subscriber(
    #         "/camera/aligned_depth_to_color/image_raw", Image, callback_depth, queue_size=10)
    # vis_sub = rospy.Subscriber(
    #         "/camera/color/image_raw", Image, callback_visible, queue_size=10)

    cinfo = rospy.wait_for_message('/camera/color/camera_info', CameraInfo, timeout=10)

    depth_sub = message_filters.Subscriber("/camera/aligned_depth_to_color/image_raw", Image)
    vis_sub = message_filters.Subscriber("/camera/color/image_raw", Image)

    ts = message_filters.TimeSynchronizer([depth_sub, vis_sub], 10)
    ts.registerCallback(callback_sync)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)
