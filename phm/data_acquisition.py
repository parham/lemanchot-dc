
import phm

import numpy as np
import time
import os
import logging
import json
import imageio
from PIL import Image
from pathlib import Path
from typing import Any
import rospy

import sensor_msgs.msg as msg
# from sensor_msgs.msg import Image, CameraInfo
import message_filters
import threading
import queue

default_config_file = 'config.json'

timestamp_label = 'timestamp'
data_label = 'data'

class Recorder (phm.Configurable) :
    def __init__(self, root_dir, main_topic_classtype, config : dict) -> None:
        super().__init__(config=config)
        # Initialize the acquisition folder
        if not hasattr(self, 'data_folder'):
            raise Exception(f'the folder for {self.name} is not defined!')
        self.data_folder = os.path.join(root_dir, self.data_folder)
        Path(self.data_folder).mkdir(parents=True,exist_ok=True)
        # The class type of main topic
        self.main_classtype = main_topic_classtype
        self.topic_subscriber = None
        self.record_thread = None
        self.record_queue = queue.Queue(maxsize=self.queue_size)
        self.sys_alive = True

    def begin_recording(self):
        # Subscribe to the topic
        if self.topic_subscriber is not None:
            return
        self.topic_subscriber = message_filters.Subscriber(self.data_topic, self.main_classtype)
        self.sys_alive = True
        self.record_thread = threading.Thread(target=self._record_loop, name=f'{self.name}_main_topic', daemon=True)
        self.record_thread.start()
        logging.info(f'Collector {self.name} is initiated and started to run ...')

    def _record_loop(self):
        try:
            while self.sys_alive:
                data = self.record_queue.get(timeout=300)
                self._record_process(data[timestamp_label], data[data_label])
        except queue.Empty as ex:
            logging.error('The data stream is not responsive! So the recorder has been terminated.')
        else:
            # Unregister the subscriber
            self.topic_subscriber.unregister()

    def _record_process(self, timestamp, data):
        pass

    def record(self, data : dict):
        if self.enable_recorder:
            if self.record_queue.full():
                self.record_queue.get()
            self.record_queue.put_nowait(data)

    def end_recording(self):
        try:
            self.sys_alive = False
            self.record_thread.join(timeout=2)
            logging.info(f'Collector {self.name} is terminated.')
        except:
            logging.error(f'{self.name} thread is failed to terminate!')

    def __str__(self) -> str:
        return f'{self.name} is handling the topic {self.data_topic}'
    
    def __repr__(self) -> str:
        return self.__str__()

class VisibleRecorder (Recorder):
    def __init__(self, root_dir, config: dict) -> None:
        super().__init__(root_dir, msg.Image, config)
    
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
        thobj = threading.Thread(target=self.record_camera_info, name=f'{self.name}_camera_info')
        thobj.start()
    
    def _record_process(self, timestamp, data):
        filename = os.path.join(self.data_folder, f'visible_{timestamp}.png')
        img = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)
        vimg = Image.fromarray(img)
        vimg.save(filename)
        
class DepthRecorder (VisibleRecorder):
    def __init__(self, root_dir, config: dict) -> None:
        super().__init__(root_dir, config)

    def _record_process(self, timestamp, data):
        filename = os.path.join(self.data_folder, f'depth_{timestamp}.png')
        img = np.frombuffer(data.data, dtype=np.uint16).reshape(data.height, data.width, -1)
        imageio.imwrite(filename, img.astype(np.uint16))

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
        self.config = phm.load_config(config_file)
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
                vobj = VisibleRecorder(self.root_dir,obj) 
                is_implemented = True
            elif name == 'depth_camera':
                vobj = DepthRecorder(self.root_dir,obj) 
                is_implemented = True
            elif name == 'sensor_imu':
                is_implemented = True
            
            if vobj is not None:
                self.collectors[name] = vobj
                vobj.begin_recording()
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

    def terminate(self):
        logging.info('LeManchot-dc is shutting down ...')
        for obj in self.collectors.values():
            obj.end_recording()