#!/usr/bin/env python

""" 
    @name phm_recorder.py   
    @info the main entry to the application (runnable execution)
    @organization: Laval University
    @professor  Professor Xavier Maldague
    @author     Parham Nooralishahi
    @email      parham.nooralishahi.1@ulaval.ca
    @equipment  FLIR A700, Intel RealSense D435i, DJI M600, DJI Manifold 2
"""

import logging
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
import message_filters
from multiprocessing import Pool, cpu_count

def main_func(argv):
    try:
        print(f'starting computations on {cpu_count()} cores')
        rospy.init_node('lemanchot-dc', argv=argv, anonymous=True)

    except rospy.ROSInitException as exp:
        logging.error('lemachot-dc system is failed!')
        logging.exception(exp)

if __name__ == '__main__':
    logging.info('=============================================================')
    logging.info('== LeManchot-DC System : Multi-Modal Data Acquisition      ==')
    logging.info('== Laval University                                        ==')
    logging.info('== @Supervisor : Professor Xavier Maldague                 ==')
    logging.info('== @Researcher : Parham Nooralishahi                       ==')
    logging.info('== @year @ 2021                                            ==')
    logging.info('=============================================================')

    main_func (sys.argv)
