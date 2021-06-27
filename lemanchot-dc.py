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
import time
import logging

import rospy
import message_filters

from phm import LeManchotDC

def callback_sync(*arg):
    dcobj = arg[-1]
    data = arg[:-1]
    packet = dict()
    for index in range(len(data)) :
        name = dcobj.collectors_name[index]
        packet[name] = data[index]
    dcobj.record(packet)

def process_dc():
    try:
        logging.info('System is started!')
        logging.info(f'starting computations on {multiprocessing.cpu_count()} cores')
        rospy.init_node('lemanchot-dc', anonymous=True)
        # rospy.on_shutdown(shutdown)
        dcobj = LeManchotDC.get_instance(config_file='./config_combo.json')

        ts = message_filters.ApproximateTimeSynchronizer(dcobj.subscribers, dcobj.buffer_size, 10)
        ts.registerCallback(callback_sync, dcobj)

        rospy.spin()
    except rospy.ROSInitException as exp:
        logging.error('lemachot-dc system is failed!')
        logging.exception(exp)
    except Exception as ex:
        logging.exception(ex)
        logging.error('lemachot-dc is terminated with an error!')

def process_gui():
    pass

if __name__ == '__main__':
    play_signal = multiprocessing.Event()
    terminate_signal = multiprocessing.Event()



    w1 = multiprocessing.Process(
        name='block',
        target=wait_for_event,
        args=(e,),
    )

    w1.start()

    w2 = multiprocessing.Process(
        name='nonblock',
        target=wait_for_event_timeout,
        args=(e, 2),
    )
    w2.start()

    print('main: waiting before calling Event.set()')
    time.sleep(3)
    e.set()
    print('main: event is set')