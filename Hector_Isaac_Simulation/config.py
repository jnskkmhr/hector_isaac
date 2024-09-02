import numpy as np
from typing import List
from dataclasses import dataclass

@dataclass
class EnvironmentConfig:
    """
    Configuration for the environment
    args:
        physics_dt: physics rate in sec
        render_dt: rendering rate in sec
    """
    physics_dt: float
    render_dt: float

@dataclass
class RobotConfig:
    """
    Configuration for the robot
    args:
        usd_path: path to the robot usd file
        prim_path: path to the robot prim
        name: name of the robot
        position: initial position of the robot
    """
    usd_path: str
    prim_path: str
    name: str
    position: np.ndarray
    ctr_sim_remap: List[int]