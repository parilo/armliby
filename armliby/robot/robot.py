from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class MotorData:
    pos: np.ndarray
    vel: np.ndarray


class JointRobot(ABC):

    @property
    @abstractmethod
    def num_joints(self) -> int:
        ...

    @abstractmethod
    def connect(self) -> None:
        ...

    @abstractmethod
    def disconnect(self) -> None:
        ...

    @abstractmethod
    def relax(self) -> None:
        ...

    @abstractmethod
    def read(self) -> MotorData:
        ...

    @abstractmethod
    def position_abs_control(
            self, 
            target_pos: np.ndarray, 
            speed_limit: Optional[np.ndarray] = None,
            torque_limit: Optional[np.ndarray] = None,
            ) -> None:
        ...

    @abstractmethod
    def velocity_control(
            self, 
            velocity: np.ndarray,
            torque_limit: Optional[np.ndarray] = None,
            ) -> np.ndarray:
        ...
