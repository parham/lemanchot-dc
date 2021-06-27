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

from kivymd.app import MDApp
from kivy.uix.screenmanager import Screen
from kivy.graphics.texture import Texture
from kivy.clock import Clock
from kivy.lang import Builder

import logging
import sys
import rospy
import numpy as np
import message_filters

import sys
import imutils
import cv2

class InternalFrame(Screen):
    """GUI internal frame class"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._pause = True

    @property
    def pause(self):
        """get pause status"""
        return self._pause

    @pause.setter
    def pause(self, pause):
        """set pause status"""
        self._pause = pause

    def on_enter(self, *args):
        """Gain focus event handler"""
        self.pause = False
        print(self.name + ' enabled')

    def on_leave(self, *args):
        """Lose focus event handler"""
        self.pause = True
        print(self.name + ' disabled')

class ViewableInternalFrame(InternalFrame):
    """Base class for gui components that show a view"""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._frame = None
        # self.no_signal_img = imutils.resize(cv2.imread(
        #     torngats.RESOURCE_NOSIGNAL_IMAGE_PATH, 0), width=640)

    def image_to_texture(self, img):
        """ Convert image to texture. """

        # convert it to texture
        if img is not None and (img.shape[0] + img.shape[1]) > 0:
            buf1 = cv2.flip(img, 0)
            buf = buf1.tostring()
            image_texture = Texture.create(
                size=(img.shape[1], img.shape[0]), colorfmt='luminance')
            image_texture.blit_buffer(
                buf, colorfmt='luminance', bufferfmt='ubyte')
            # display image from the texture
            return image_texture

        return None

    def colorimage_to_texture(self, img):
        """ Convert RGB image to texture"""

        # convert it to texture
        if img is not None and (img.shape[0] + img.shape[1]) > 0:
            buf1 = cv2.flip(img, 0)
            buf = buf1.tostring()
            image_texture = Texture.create(
                size=(img.shape[1], img.shape[0]), colorfmt='bgr')
            image_texture.blit_buffer(
                buf, colorfmt='bgr', bufferfmt='ubyte')
            # display image from the texture
            return image_texture

        return None

    def load_next_frame(self):
        """ Get the next frame from the queue"""
        pass
        # qbatch = module.QueueBatch.get_instance()
        # queue = qbatch[gui_queue_name]

        # if not self.pause and not queue.empty():
        #     self._frame = queue.get()

    def get_vi_data(self):
        visimg = None
        thimg = None
        thorig = None

        # if self._frame is not None:
        #     visimg = self._frame[dc.visible_label] if dc.visible_label in self._frame else self.no_signal_img
        #     thimg = self._frame[dc.thermal_label] if dc.thermal_label in self._frame else self.no_signal_img
        #     thorig = self._frame[dc.thermal_orig_label] if dc.thermal_orig_label in self._frame else self.no_signal_img

        return visimg, thimg, thorig

class LeManchotFrame (ViewableInternalFrame):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self._updateview_clock = None

    def callback_sync(self, *arg):
        dcobj = arg[-1]
        data = arg[:-1]
        packet = dict()
        for index in range(len(data)) :
            name = self.dcobj.collectors_name[index]
            packet[name] = data[index]
        dcobj.record(packet)

    def on_enter(self, *args):
        super().on_enter(*args)

        # Update the frames from the queue (30 times/second)
        if self._updateview_clock is None:
            self._updateview_clock = Clock.schedule_interval(
                self.update_viewers, 1.0 / 30)

    def on_leave(self, *args):
        super().on_leave(*args)
        if self._updateview_clock is not None:
            self._updateview_clock.cancel()
            self._updateview_clock = None

    def update_viewers(self, dt):
        pass
        # Block till the data is available
        # Load Next Frame
        # self.load_next_frame()
        # visimg, _, thorig = self.get_vi_data()

        # if visimg is not None and thorig is not None:
        #     visimg = calib_obj.fusion(visimg, thorig)
        #     resvis = cv2.addWeighted(visimg, 0.3, thorig, 0.7, 0.1)
        #     vis_texture = self.image_to_texture(resvis)
        #     if vis_texture is not None:
        #         self.ids['batman_visible_camera_viewer'].texture = vis_texture
        #     # COLORMAP_RAINBOW COLORMAP_JET
        #     resth = cv2.applyColorMap(thorig, cv2.COLORMAP_JET)
        #     inf_texture = self.colorimage_to_texture(resth)
        #     if inf_texture is not None:
        #         self.ids['batman_infrared_camera_viewer'].texture = inf_texture

class LeManchotApp(MDApp):

    def __init__(self, dc, **kwargs):
        super().__init__(**kwargs)
        self.dcobj = dc

    def build(self):
        # "Blue"  # "Purple", "Red", "Blue", "Green"
        self.theme_cls.primary_palette = 'BlueGray'
        # "Light"  # "Dark", "Light"
        self.theme_cls.theme_style = 'Dark'
        return LeManchotFrame()

Builder.load_file('phm_gui.kv')

if __name__ == '__main__':
    print('=============================================================')
    print('== LeManchot-DC System : Multi-Modal Data Acquisition      ==')
    print('== Laval University                                        ==')
    print('== @Supervisor : Professor Xavier Maldague                 ==')
    print('== @Researcher : Parham Nooralishahi                       ==')
    print('== @year @ 2021                                            ==')
    print('=============================================================')

    app = LeManchotApp()
    app.run()