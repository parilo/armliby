from typing import Optional

import numpy as np
from armliby.robot.joint_limits import JointLimits

from ..robot import JointRobot, MotorData


class VirtualPosRobot(JointRobot):
    def __init__(
            self,
            start_joints: np.ndarray,
            joint_limits: Optional[JointLimits] = None,
            ):
        """
        Initialize the virtual robot.

        Args:
            start_joints: The initial joint positions of the robot.
            joint_limits: Optional joint limits for the robot.
        """
        self._joint_count = len(start_joints)
        self._current_jpos = start_joints
        self._joint_limits = joint_limits
        self._connected = False

    @property
    def num_joints(self) -> int:
        return self._joint_count

    def connect(self) -> None:
        if self._connected:
            print("Robot is already connected.")
            return
        self._connected = True

    def disconnect(self) -> None:
        if not self._connected:
            print("Robot is already disconnected.")
            return
        self._connected = False

    def relax(self) -> None:
        """Set the robot to a relaxed state."""
        ...

    def read(self) -> MotorData:
        """Read the current state of the robot."""
        return MotorData(pos=self._current_jpos.copy(), vel=np.zeros(self._joint_count))

    def position_abs_control(
            self, 
            target_pos: np.ndarray,
            torque_limit: Optional[np.ndarray] = None,
            ) -> None:
        """Move the robot to the specified target position."""
        if not self._connected:
            raise RuntimeError("VirtualRobot is not connected.")

        if self._joint_limits is not None:
            target_pos = self._joint_limits.process(target_pos)

        self._current_jpos = target_pos
    
    def velocity_control(
            self, 
            velocity: np.ndarray,
            torque_limit: Optional[np.ndarray] = None,
            ) -> np.ndarray:
        raise NotImplementedError("Velocity control is not supported for VirtualPosRobot.")
