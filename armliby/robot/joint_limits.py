from typing import List, Optional

import numpy as np
from armliby.urdf_parser import URDFParser


class JointLimits:

    def __init__(
            self,
            lower: np.ndarray,
            upper: np.ndarray,
            ) -> None:
        """
        Initialize the joint limits.

        Args:
            lower: The lower limits for each joint.
            upper: The upper limits for each joint.
        """
        self.lower = lower
        self.upper = upper

    def process(self, joints: np.ndarray) -> np.ndarray:
        return np.clip(
            joints,
            self.lower,
            self.upper,
        )

    @staticmethod
    def from_urdf(
            urdf_path: str,
            skip_links: Optional[List[int]] = None,
            skip_joints: Optional[List[int]] = None,
            ) -> 'JointLimits':
        """
        Args:
            urdf_path (str): Path to the URDF file.
            skip_links (Optional[List[int]], optional): _description_. Defaults to None.
            skip_joints (Optional[List[int]], optional): _description_. Defaults to None.
        """

        parser = URDFParser(
            urdf_path=urdf_path,
            skip_links=skip_links,
            skip_joints=skip_joints,
        )
        jlimits = parser.get_joint_pos_limits()
        return JointLimits(
            lower=np.rad2deg(np.array(jlimits[0])),
            upper=np.rad2deg(np.array(jlimits[1])),
        )
