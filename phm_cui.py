#!/usr/bin/env python

""" 
    @name phm_main.py   
    @info the main entry to the application (runnable execution)
    @organization: Laval University
    @professor  Professor Xavier Maldague
    @author     Parham Nooralishahi
    @email      parham.nooralishahi.1@ulaval.ca
    @equipment  FLIR A700, Intel RealSense D435i, DJI M600, DJI Manifold 2
"""

from consolemenu import ConsoleMenu, SelectionMenu
from consolemenu.items import FunctionItem, SubmenuItem, CommandItem

import logging
from phm.data_acquisition import LeManchotDC
import sys
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
import message_filters
from multiprocessing import cpu_count



def callback_sync(*arg):
    dcobj = arg[-1]
    data = arg[:-1]
    packet = dict()
    for index in range(len(data)) :
        name = dcobj.collectors_name[index]
        packet[name] = data[index]
    dcobj.record(packet)
    

def shutdown():
    logging.info('Node is shutting down ...')
    

def main_func(argv):
    try:
        logging.info('System is started!')
        logging.info(f'starting computations on {cpu_count()} cores')
        rospy.init_node('lemanchot-dc', argv=argv, anonymous=True)
        rospy.on_shutdown(shutdown)

        # dcobj = LeManchotDC.get_instance(config_file='./config.json')
        dcobj = LeManchotDC.get_instance(config_file='./config_combo.json')

        ts = message_filters.ApproximateTimeSynchronizer(dcobj.subscribers, dcobj.buffer_size, 10)
        ts.registerCallback(callback_sync, dcobj)

        # Create the menu
        title = 'LeManchot-DC'
        subtitle = ''' 
            LeManchot is a multimodal data platform including data acquisition and data transmission components.
            Developed by Parham Nooralishahi - parham.nooralishahi.1@ulaval.ca
            Supervisor : Professor Xavier Maldague
            @2021
        '''
        prologue = 'You can select the following options for controling DC process'
        epilogue = '@ULAVAL'

        menu = ConsoleMenu(title, subtitle=subtitle, prologue_text=prologue, epilogue_text=epilogue)
        # menu.clear_screen()
        menu.append_item(FunctionItem("Pause recording", dcobj.pause, menu=menu))
        menu.append_item(FunctionItem("Resume recording", dcobj.resume, menu=menu))
        menu.append_item(FunctionItem("Stop recording", dcobj.stop, menu=menu))
        # Finally, we call show to show the menu and allow the user to interact
        menu.show()

        logging.info('System is terminated!')
    except rospy.ROSInitException as exp:
        logging.error('lemachot-dc system is failed!')
        logging.exception(exp)


if __name__ == '__main__':
    print('=============================================================')
    print('== LeManchot-DC System : Multi-Modal Data Acquisition      ==')
    print('== Laval University                                        ==')
    print('== @Supervisor : Professor Xavier Maldague                 ==')
    print('== @Researcher : Parham Nooralishahi                       ==')
    print('== @year @ 2021                                            ==')
    print('=============================================================')

    main_func (sys.argv)