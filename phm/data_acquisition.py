

import cv2
import numpy as np
import time
import os
import logging
import json
import imageio
import rospy
import csv

from phm import Configurable, load_config
from PIL import Image
from pathlib import Path
from typing import Any

from realsense2_camera.msg import IMUInfo

import dynamic_reconfigure.client as dclient
import sensor_msgs.msg as msg
import message_filters
import threading
import queue

default_config_file = 'config.json'

timestamp_label = 'timestamp'
data_label = 'data'

class Recorder (Configurable, threading.Thread) :
    def __init__(self, root_dir, name, config : dict) -> None:
        threading.Thread.__init__(self, name=f'{name}_main_topic', daemon=True)
        Configurable.__init__(self, config=config)
        # Initialize the acquisition folder
        if not hasattr(self, 'data_folder'):
            raise Exception(f'the folder for {self.name} is not defined!')
        self.data_folder = os.path.join(root_dir, self.data_folder)
        Path(self.data_folder).mkdir(parents=True,exist_ok=True)

        self.record_thread = None
        self.record_queue = queue.Queue(maxsize=self.queue_size)
        self.sys_alive = True

        self.__flag = threading.Event() # The flag used to pause the thread
        self.__flag.clear() # Set to True
        self.__running = threading.Event() # Used to stop the thread identification
        self.__running.set() # Set running to True

    def start(self):
        threading.Thread.start(self)
        self.begin_recording()

    def run(self):
        try:
            while self.__running.isSet():
                self.__flag.wait()
                data = self.record_queue.get(timeout=300)
                self._record_process(data[timestamp_label], data[data_label])
        except queue.Empty as ex:
            logging.error('The data stream is not responsive! So the recorder has been terminated.')
        else:
            # Unregister the subscriber
            self.topic_subscriber.unregister()

    def begin_recording(self):
        self.sys_alive = True
        logging.info(f'Collector {self.name} is initiated and started to run ...')

    def pause(self):
        self.__flag.clear() # Set to False to block the thread
        logging.info(f'{self.name} collector is paused!')

    def resume(self):
        self.__flag.set() # Set to True, let the thread stop blocking
        logging.info(f'{self.name} collector is resumed!')

    def stop(self):
        self.end_recording()
        self.__flag.set() # Resume the thread from the suspended state, if it is already suspended
        self.__running.clear() # Set to False
        logging.info(f'Collector {self.name} is terminated.')

    def _record_process(self, timestamp, data):
        pass

    def record(self, data : dict):
        if self.enable_recorder and self.sys_alive:
            if self.record_queue.full():
                self.record_queue.get()
            self.record_queue.put_nowait(data)

    def end_recording(self):
        pass

    def __str__(self) -> str:
        return f'{self.name} is handling the topic {self.data_topic}'
    
    def __repr__(self) -> str:
        return self.__str__()

class DataRecorder (Recorder):
    def __init__(self, root_dir, name, main_topic_classtype, config : dict) -> None:
        super().__init__(root_dir, name, config)
        # The class type of main topic
        self.main_classtype = main_topic_classtype
        self.topic_subscriber = None

    def begin_recording(self):
        super().begin_recording()
        # Subscribe to the topic
        if self.topic_subscriber is not None:
            return
        self.topic_subscriber = message_filters.Subscriber(self.data_topic, self.main_classtype)

class RealSenseIMURecorder (DataRecorder):
    def __init__(self, root_dir, name, config: dict) -> None:
        super().__init__(root_dir, name, msg.Imu, config)
        self.filename = None
        self.records = list()

    def record_camera_info(self, *args: Any, **kwargs: Any) :
        # Collect a packet from camera info topic
        cinfo = rospy.wait_for_message(self.info_topic, IMUInfo, timeout=10)
        # Extract the camera calibration fields
        obj = dict()
        obj['bias_variances'] = cinfo.bias_variances
        obj['data'] = cinfo.data
        obj['noise_variances'] = cinfo.noise_variances
        # Write the info
        with open(os.path.join(self.data_folder, f'{self.name}_info.json'), "w") as outfile: 
            json.dump(obj, outfile, indent = 4)
        
    def begin_recording(self):
        super().begin_recording()
        # Collect Camera Information
        if hasattr(self, 'info_topic'):
            thobj = threading.Thread(target=self.record_camera_info, name=f'{self.name}_info')
            thobj.start()
        timestamp = int(time.time() * 1000.0)
        self.filename = os.path.join(self.data_folder, f'{self.name}_{timestamp}.csv')

    def _record_process(self, timestamp, data):
        if len(self.records) >= self.flush_size:
            fexist = os.path.exists(self.filename)
            ref = self.records[0]
            with open(self.filename, 'a' if fexist else 'w', newline='') as outcsv:
                writer = csv.DictWriter(outcsv, delimiter =';', fieldnames = list(ref.keys()))
                if not fexist:
                    writer.writeheader()
                writer.writerows(self.records)
                self.records = list()
        else:
            tmp = dict()
            tmp['angular_velocity'] = [data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z]
            tmp['angular_velocity_covariance'] = data.angular_velocity_covariance
            tmp['linear_acceleration'] = [data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z]
            tmp['linear_acceleration_covariance'] = data.linear_acceleration_covariance
            tmp['orientation_w'] = data.orientation.w
            tmp['orientation_x'] = data.orientation.x
            tmp['orientation_y'] = data.orientation.y
            tmp['orientation_z'] = data.orientation.z
            tmp['orientation_covariance'] = data.orientation_covariance
            tmp[timestamp_label] = timestamp
            self.records.append(tmp)

class VisibleRecorder (DataRecorder):
    def __init__(self, root_dir, name, config: dict) -> None:
        super().__init__(root_dir, name, msg.Image, config)
    
    def record_camera_info(self, *args: Any, **kwargs: Any) :
        # Collect a packet from camera info topic
        cinfo = rospy.wait_for_message(self.info_topic, msg.CameraInfo, timeout=10)
        # Extract the camera calibration fields
        obj = dict()
        obj['D'] = cinfo.D
        obj['K'] = cinfo.K
        obj['P'] = cinfo.P
        obj['R'] = cinfo.R
        obj['binning_x'] = cinfo.binning_x
        obj['binning_y'] = cinfo.binning_y
        obj['distortion_model'] = cinfo.distortion_model
        obj['height'] = cinfo.height
        obj['width'] = cinfo.width
        # Write the info
        with open(os.path.join(self.data_folder, 'camera_info.json'), "w") as outfile: 
            json.dump(obj, outfile, indent = 4)

    def begin_recording(self):
        super().begin_recording()
        # Collect Camera Information
        if hasattr(self, 'info_topic'):
            thobj = threading.Thread(target=self.record_camera_info, name=f'{self.name}_camera_info')
            thobj.start()
    
    def _record_process(self, timestamp, data):
        filename = os.path.join(self.data_folder, f'visible_{timestamp}.png')
        img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        # TODO: Change the following lines with : imageio.imwrite(filename, img.astype(np.uint8))
        vimg = Image.fromarray(img)
        vimg.save(filename)
        
class DepthRecorder (VisibleRecorder):
    def __init__(self, root_dir, name, config: dict) -> None:
        super().__init__(root_dir, name, config)

    def _record_process(self, timestamp, data):
        filename = os.path.join(self.data_folder, f'depth_{timestamp}.png')
        img = np.frombuffer(data.data, dtype=np.uint16).reshape(data.height, data.width, -1)
        imageio.imwrite(filename, img.astype(np.uint16))

class ThermalRecorder (VisibleRecorder):
    def __init__(self, root_dir, name, config: dict) -> None:
        super().__init__(root_dir, name, config)
        self.pixel_format = None
        self.image_mode = None

    def begin_recording(self):
        super().begin_recording()
        dclient.Client(self.config_node, timeout=10, config_callback=self.config_callback)
    
    def config_callback(self, config):
        self.pixel_format = config['frame_pixel_format']
        self.image_mode = config['image_mode']

    def _record_process(self, timestamp, data):
        filename = os.path.join(self.data_folder, f'thermal_{timestamp}.png')
        if self.pixel_format is not None and self.image_mode == 'Thermal':
            img = None
            if self.pixel_format == 'Mono8':
                img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
            if self.pixel_format == 'Mono16':
                img = np.frombuffer(data.data, dtype=np.uint16).reshape(data.height, data.width, -1)
                # imageio.imwrite(filename, img.astype(np.uint16))
            if img is not None:
                if self.flip:
                    img = cv2.flip(img, -1)
                imageio.imwrite(filename, img)

class LeManchotDC:

    __instance = None

    def __init__(self) -> None:
        if LeManchotDC.__instance != None:
            raise Exception('The class has already initialized!')
        else:
            LeManchotDC.__instance = self

        self.config = None
        self.root_dir = None
        self.buffer_size = 1
        self.collectors = dict()
        self.collectors_name = list()
        self.subscribers = list()
        self.__last_time = time.time()

    def get_instance(config_file = None):
        """ Create or get the instance of the data acquisition component """
        if LeManchotDC.__instance == None:
            obj = LeManchotDC()
            obj._initialize(config_file=config_file)
            logging.info('LeManchot-DC is initialized.')
        return LeManchotDC.__instance
    
    def _initialize(self, config_file = None):
        # Load the configuration
        self.config = load_config(config_file)
        # Initialize the root directory
        self.root_dir = self.config['root_dir'] if 'root_dir' in self.config else os.path.join(os.getcwd(), 'data')
        Path(self.root_dir).mkdir(parents=True,exist_ok=True)
        # Initialize the buffer size
        self.buffer_size = self.config['buffer_size'] if 'buffer_size' in self.config else 5
        # Initialize the time lack
        self.time_lack = self.config['time_lack'] if 'time_lack' in self.config else 0
        # Initialize collectors
        cols_config = self.config['collectors']
        if cols_config is None or not cols_config:
            raise Exception('Collectors are not determined!')
        self.initialize_collectors(cols_config)
        
    def initialize_collectors(self, cols_config : list) -> dict :
        # Create the instances of targeted collectors
        for obj in cols_config:
            name = obj['name']
            vobj = None

            if name in self.collectors:
                continue

            # Selecting proper collectors
            is_implemented = False
            if name == 'visible_camera':
                vobj = VisibleRecorder(self.root_dir, name, obj) 
                is_implemented = True
            elif name == 'depth_camera':
                vobj = DepthRecorder(self.root_dir, name, obj) 
                is_implemented = True
            elif name == 'thermal_camera':
                vobj = ThermalRecorder(self.root_dir, name, obj)
                is_implemented = True
            elif name == 'gyro_imu':
                vobj = RealSenseIMURecorder(self.root_dir, name, obj)
                is_implemented = True
            elif name == 'accel_imu':
                vobj = RealSenseIMURecorder(self.root_dir, name, obj)
                is_implemented = True
            
            if vobj is not None:
                self.collectors[name] = vobj
                vobj.start()
                self.subscribers.append(vobj.topic_subscriber)
                self.collectors_name.append(name)
            elif not is_implemented:
                raise NotImplementedError()

    def record(self, data : dict):
        timestamp = int(time.time() * 1000.0)
        if (timestamp - self.__last_time) > self.time_lack:
            for key, value in data.items():
                self.collectors[key].record({
                    timestamp_label : timestamp,
                    data_label : value
                })
            self.__last_time = timestamp

    def callback_sync(self,*arg):
        dcobj = arg[-1]
        data = arg[:-1]
        packet = dict()
        for index in range(len(data)) :
            name = dcobj.collectors_name[index]
            packet[name] = data[index]
        dcobj.record(packet)

    def start(self):
        ts = message_filters.ApproximateTimeSynchronizer(self.subscribers, self.buffer_size, 10)
        ts.registerCallback(self.callback_sync, self)

    def pause(self):
        logging.info('LeManchot-dc is paused ...')
        for obj in self.collectors.values():
            obj.pause()
    
    def resume(self):
        logging.info('LeManchot-dc is resumed ...')
        for obj in self.collectors.values():
            obj.resume()

    def stop(self):
        logging.info('LeManchot-dc is shutting down ...')
        for obj in self.collectors.values():
            obj.stop()

