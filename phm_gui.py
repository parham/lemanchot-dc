#!/usr/bin/env python

""" 
    @name phm_gui.py   
    @info the main entry to the gui application (runnable execution)
    @organization: Laval University
    @professor  Professor Xavier Maldague
    @author     Parham Nooralishahi
    @email      parham.nooralishahi.1@ulaval.ca
    @equipment  FLIR A700, Intel RealSense D435i, DJI M600, DJI Manifold 2
"""


import phm.gui as gui

if __name__ == '__main__':
    print('=============================================================')
    print('== LeManchot-DC System : Multi-Modal Data Acquisition      ==')
    print('== Laval University                                        ==')
    print('== @Supervisor : Professor Xavier Maldague                 ==')
    print('== @Researcher : Parham Nooralishahi                       ==')
    print('== @year @ 2021                                            ==')
    print('=============================================================')


    gui.run_gui(config_file='./config_combo.json')
