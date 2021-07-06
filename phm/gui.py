
import multiprocessing
from kivymd.app import MDApp
from kivy.uix.screenmanager import Screen
from kivy.graphics.texture import Texture
from kivy.clock import Clock
from kivy.lang import Builder
from kivy.config import Config
from kivy.core.window import Window
Config.set('graphics', 'window_state', 'maximized')

from phm import load_config
import logging
import sys
import os
import rospy
import numpy as np
import message_filters

import psutil
import sys
import imutils
import cv2
import rospy
from PIL import Image

__gui_template_path = './phm/gui.kv'


def get_directory_size(start_path):
    """Returns the `directory` size in bytes."""
    total_size = 0
    try:
        for path, dirs, files in os.walk(start_path):
            for f in files:
                fp = os.path.join(path, f)
                total_size += os.path.getsize(fp)
    except PermissionError:
        # if for whatever reason we can't open the folder, return 0
        return 0
    return '%.2f GB' % (float(total_size) / 1073741824.0)

class LeManchotFrame (Screen):

    __no_signal_img = './resources/no_signal.png'

    def __init__(self, config, queue_visible = None, queue_thermal = None, record_signal = None, terminate_signal = None):
        super().__init__()
        self.collectors = dict()
        self.sensor_index = 0
        self._updateview_clock = Clock.schedule_interval(
            self.update_viewers, 1.0 / 30)
        self._updateview_clock = Clock.schedule_interval(
            self.update_info, 1.0)
        Clock.schedule_once(self.maximize, 0)
        self.queue_visible = queue_visible if queue_visible is not None else multiprocessing.Queue()
        self.queue_thermal = queue_thermal if queue_thermal is not None else multiprocessing.Queue()
        self.record_signal = record_signal 
        self.terminate_signal = terminate_signal
        self.vis_or_th = True
        self.no_signal_img = self.colorimage_to_texture(imutils.resize(cv2.imread(self.__no_signal_img, 0), width=640)) 

    def maximize (self, dt):
        Window.maximize()

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

    def next_frame(self):
        texture = None

        if not self.queue_visible.empty():
            img = self.queue_visible.get()
            texture_vis = self.colorimage_to_texture(img)
        else:
            texture_vis = self.no_signal_img

        if not self.queue_thermal.empty():
            img = self.queue_thermal.get()
            img = img.astype('uint8')
            texture_th = self.image_to_texture(img)
        else:
            texture_th = self.no_signal_img

        return texture_vis, texture_th

    def update_info(self, dt):
        self.ids['phm_cpu_indicator'].text = f'{psutil.cpu_percent()} %'
        self.ids['phm_storage_indicator'].text = get_directory_size('./recordings')

    def update_viewers(self, dt):
        # Load next frame
        texture_vis, texture_th = self.next_frame()

        if self.vis_or_th:
            self.ids['phm_camera_main'].texture = texture_vis
            self.ids['phm_camera_secondary'].texture = texture_th
        else:
            self.ids['phm_camera_secondary'].texture = texture_vis
            self.ids['phm_camera_main'].texture = texture_th

    def switch_view(self):
        self.vis_or_th = not self.vis_or_th
        print('switch')
    
    def pause_dc(self):
        self.record_signal.clear()
        self.ids['recording_chip'].text = 'NOT RECORDING'
        print('lemanchot-dc is paused')
    
    def resume_dc(self):
        self.record_signal.set()
        self.ids['recording_chip'].text = 'RECORDING'
        print('lemanchot-dc is resumed')

class LeManchotApp(MDApp):

    def __init__(self, config, queue_visible = None, queue_thermal = None, record_signal = None, terminate_signal = None):
        super().__init__()
        self.app_config = config
        self.queue_visible = queue_visible
        self.queue_thermal = queue_thermal
        self.record_signal = record_signal
        self.terminate_signal = terminate_signal

    def build(self):
        # "Blue"  # "Purple", "Red", "Blue", "Green"
        self.theme_cls.primary_palette = 'BlueGray'
        # "Light"  # "Dark", "Light"
        self.theme_cls.theme_style = 'Dark'
        return LeManchotFrame(self.app_config, self.queue_visible, self.queue_thermal, self.record_signal, self.terminate_signal)

    def on_stop(self):
        self.terminate_signal.set()
        return super().on_stop()


def run_gui(config_file, queue_visible = None, queue_thermal = None, record_signal = None, terminate_signal = None):
    config = load_config(config_file)
    Builder.load_file(__gui_template_path)
    app = LeManchotApp(config, queue_visible, queue_thermal, record_signal, terminate_signal)
    app.run()