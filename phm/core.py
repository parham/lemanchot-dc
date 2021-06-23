
""" 
    @name core.py   
    @info all the base or configuration classes and components
    @organization: Laval University
    @professor  Professor Xavier Maldague
    @author     Parham Nooralishahi
    @email      parham.nooralishahi.1@ulaval.ca
"""

import time
import os
import functools
import logging
import logging.config
import json
import yaml
from pathlib import Path

default_log_config_file = 'log_config.yml'

def exception_logger(function):
    """
    A decorator that wraps the passed in function and logs 
    exceptions should one occur
    """
    @functools.wraps(function)
    def wrapper(*args, **kwargs):
        try:
            return function(*args, **kwargs)
        except:
            # log the exception
            err = "There was an exception in  " + function.__name__
            logging.exception(err)
            # re-raise the exception
            raise
    return wrapper

def initialize_log():
    os.environ['ROS_PYTHON_LOG_CONFIG_FILE'] = default_log_config_file
    Path('logs').mkdir(parents=True, exist_ok=True)
    """Initialize the log configuration"""
    if os.path.isfile(default_log_config_file):
        with open(default_log_config_file, 'r') as f:
            config = yaml.safe_load(f.read())
            logging.config.dictConfig(config)
            logging.getLogger().setLevel(logging.INFO)
        logging.info(
            'Logging is configured based on the defined configuration file.')
    else:
        logging.error('the logging configuration file does not exist')

@exception_logger
def load_config(config_file):
    config = dict()
    with open(config_file, 'r') as cfile:
        config = json.load(cfile)
    return config

def save_config(config, config_file):
    with open(config_file, 'w') as cfile:
        json.dump(config, config_file)

class Configurable:
    def __init__(self, config : dict) -> None:
        for key, value in config.items():
            self.__setattr__(key,value)

    def as_dict(self) -> dict:
        return self.config


