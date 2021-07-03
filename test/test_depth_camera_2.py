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


def callback_sync(*data):
    depth = data[0]
    viz = data[1]
    try:
        vimg = np.frombuffer(viz.data, dtype=np.uint8).reshape(viz.height, viz.width, -1)
        dimg = np.frombuffer(depth.data, dtype=np.uint8).reshape(depth.height, depth.width, -1)
    except Exception as ex:
        print(ex)

    cv2.imshow("Visible Window", vimg)
    cv2.imshow("Depth Window", dimg)
    cv2.waitKey(10)

def shutdown():
    rospy.loginfo('Node is shutting down ...')
    cv2.destroyAllWindows()

def main(args):
    # print('Node Test is being started ...')
    logging.info('Node Test is being started ...')
    rospy.init_node('image_converter', anonymous=True)
    rospy.on_shutdown(shutdown)

    depth_sub = message_filters.Subscriber("/phm/thermal_camera/image", Image)
    # depth_sub = message_filters.Subscriber("/phm/depth_camera/aligned_depth_to_color/image_raw", Image)
    vis_sub = message_filters.Subscriber("/phm/depth_camera/color/image_raw", Image)

    ts = message_filters.ApproximateTimeSynchronizer([depth_sub, vis_sub], 20, 5, allow_headerless=True)
    # ts = message_filters.sync_policies.ApproximateTimeSynchronizer([depth_sub, vis_sub], 20, 0.5)
    ts.registerCallback(callback_sync)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    # cv2.destroyAllWindows()



if __name__ == '__main__':
    main(sys.argv)
