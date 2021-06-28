#!/usr/bin/env python

""" 
    @name lemanchot.py   
    @info the main entry to whole program (runnable execution)
    @organization: Laval University
    @professor  Professor Xavier Maldague
    @author     Parham Nooralishahi
    @email      parham.nooralishahi.1@ulaval.ca
    @equipment  FLIR A700, Intel RealSense D435i, DJI M600, DJI Manifold 2
"""

import multiprocessing
from multiprocessing import process
import queue
import time
import logging

import cv2
import numpy as np
import rospy
import message_filters
import sensor_msgs.msg as msg

config_file = './config_combo.json'

from phm import LeManchotDC
from phm.gui import run_gui

def shutdown():
    logging.info('ros node is terminating!')

def process_dc(record_signal, terminate_signal):
    try:
        logging.info('System is started!')
        logging.info(f'starting computations on {multiprocessing.cpu_count()} cores')
        rospy.init_node('lemanchot_dc')
        rospy.on_shutdown(shutdown)
        dcobj = LeManchotDC.get_instance(config_file=config_file)
        dcobj.start()

        while not rospy.is_shutdown():
            if record_signal.is_set():
                dcobj.resume()
            else:
                dcobj.pause()
            
            if terminate_signal.is_set():
                dcobj.stop()
                break
            time.sleep(0.2)

        rospy.signal_shutdown('lemanchot-dc is shutting down')

    except rospy.ROSInitException as exp:
        logging.error('lemachot-dc system is failed!')
        logging.exception(exp)
    except Exception as ex:
        logging.exception(ex)
        logging.error('lemachot-dc is terminated with an error!')


queue_visible = multiprocessing.Queue(maxsize=20)
queue_thermal = multiprocessing.Queue(maxsize=20)

def callback_visible(data):
    if queue_visible.full():
        queue_visible.get()
    img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    queue_visible.put(img)

def callback_thermal(data):
    if queue_thermal.full():
        queue_thermal.get()
    img = None
    if data.encoding == 'mono8' :
        img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
    elif data.encoding == 'mono16' :
        img = np.frombuffer(data.data, dtype=np.uint16).reshape(data.height, data.width, -1)
    if img is not None:
        img = cv2.flip(img, -1)
        queue_thermal.put(img)

if __name__ == '__main__':
    try:
        record_signal = multiprocessing.Event()
        terminate_signal = multiprocessing.Event()
        #  Start the LeManchot-DC subsystem
        dc_worker = multiprocessing.Process(
            name='lemanchot-dc',
            target=process_dc,
            args=(record_signal,terminate_signal),
        )
        dc_worker.start()

        rospy.init_node('lemanchot_gui')
        rospy.Subscriber('/phm/depth_camera/color/image_raw', msg.Image, callback=callback_visible)
        rospy.Subscriber('/phm/thermal_camera/image', msg.Image, callback=callback_thermal)

        run_gui(config_file, queue_visible, queue_thermal, record_signal, terminate_signal) 

    except Exception as ex:
        logging.info('System is terminating because of an exception!')
        logging.exception(ex)
        # print(ex)
    finally:
        # dc_worker.terminate()
        # dc_worker.join(timeout=2)
        # dc_worker.kill()
        exit()
