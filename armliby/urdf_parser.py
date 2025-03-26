import os
from typing import Dict, List, Optional, Tuple

from yourdfpy import URDF


class URDFParser:
    def __init__(
            self, 
            urdf_path: str,
            skip_links: Optional[List[int]] = None,
            skip_joints: Optional[List[int]] = None,
            ) -> None:
        """
        Parse the URDF file.

        Args:
            urdf_path (str): Path to the URDF file.
            skip_links (Optional[List[int]], optional): List of link indices to skip. Defaults to None.
            skip_joints (Optional[List[int]], optional): List of joint indices to skip. Defaults to None.
        """
        self._urdf_path: str = urdf_path
        self._urdf: URDF = URDF.load(urdf_path)
        self._skip_links = set() if skip_links is None else set(skip_links)
        self._skip_joints = set() if skip_joints is None else set(skip_joints)

    def get_link_stl_map(self) -> Dict[str, str]:
        """
        Return the link to STL file mapping.

        Returns:
            Dict[str, str]: A dictionary mapping link names to STL file paths.
        """
        return {
            link.name: os.path.normpath(os.path.join(
                os.path.dirname(self._urdf_path),
                link.visuals[0].geometry.mesh.filename),
            )
            for ind, link
            in enumerate(self._urdf.robot.links)
            if ind not in self._skip_links and len(link.visuals) > 0
        }

    def get_links(self) -> List[str]:
        """
        Get the list of link names.

        Returns:
            List[str]: A list of link names.
        """
        return [
            l.name 
            for ind, l in enumerate(self._urdf.robot.links)
            if ind not in self._skip_links
        ]

    def get_joint_pos_limits(self) -> Tuple[List[float], List[float]]:
        """
        Get joint limits.

        Returns:
            Tuple[List[float], List[float]]: lists of min joint value and max joint value
        """
        lower = []
        upper = []
        for ind, joint in enumerate(self._urdf.robot.joints):
            if ind not in self._skip_joints:
                lower.append(joint.limit.lower)
                upper.append(joint.limit.upper)
        return lower, upper
