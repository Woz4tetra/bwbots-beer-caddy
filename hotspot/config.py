import os
from dataclasses import dataclass

import yaml


@dataclass
class Config:
    wifi_interface: str
    hotspot_name: str
    waiting_period: float
    main_service: str


def base_dir():
    return os.path.dirname(os.path.abspath(__file__))


def load_config():
    with open(os.path.join(base_dir(), "config.yaml")) as file:
        config = yaml.safe_load(file)
    return Config(**config)
