from typing import Dict, Optional

import numpy as np
import pytorch_kinematics as pk
from scipy.spatial.transform import Rotation as R


class Kinematics:
    def __init__(
            self,
            urdf_path: str,
            end_link_name: str,
            mod_matrix: Optional[np.ndarray] = None,
            ) -> None:
        """
        Initialize forward and inverse kinematics solvers.

        Args:
            urdf_path: The path to the URDF file.
            end_link_name: The name of the end effector link.
            mod_matrix: The modification matrix for the Jacobian. 
                Useful if robot dof is less than 6.
                Jacobian and cartesian delta is transformed by mod_matrix.
                dx = mod_matrix @ dx
                jac = mod_matrix @ jac
        """
        # Load robot description from URDF and specify end effector link
        with open(urdf_path, "rb") as f:
            urdf_content = f.read()

        self._mod_matrix = mod_matrix

        self.end_link_name = end_link_name
        self._chain = pk.build_serial_chain_from_urdf(
            data=urdf_content,
            end_link_name=end_link_name,
        )

    @property
    def num_dof(self) -> int:
        return len(self._chain.get_joints())

    def fk(self, js: np.ndarray) -> Dict[str, np.ndarray]:
        """
        Perform forward kinematics and get transforms for all links.

        Args:
            js: The joint angles.

        Returns:
            A dictionary of transforms.
        """
        ret = self._chain.forward_kinematics(js, end_only=False)
        ret = {key: val.get_matrix().cpu().detach().numpy()[0] for key, val in ret.items()}
        return ret

    def dj(
            self,
            js: np.ndarray,
            dx: np.ndarray,
            ) -> np.ndarray:
        """
        Compute the joint deltas to achieve a desired end effector cartesian delta.

        Args:
            js: The current joint angles. In radians.
            dx: The desired end effector cartesian delta.

        Returns:
            The joint deltas.
        """
        jac = self._chain.jacobian(js).detach().numpy()[0]
        if self._mod_matrix is not None:
            jac = self._mod_matrix @ jac
            dx = self._mod_matrix @ dx

        jac_inv = np.linalg.pinv(jac)
        dj = jac_inv @ dx
        return dj

    def dx(
            self,
            js1: np.ndarray,
            js2: np.ndarray,
            ) -> np.ndarray:
        """
        Compute the cartesian delta between two joint configurations.

        Args:
            js1: The first joint configuration.
            js2: The second joint configuration.

        Returns:
            The cartesian delta.
        """
        
        x1 = self.fk(js1)[self.end_link_name]
        x2 = self.fk(js2)[self.end_link_name]
        d_translation = x2[:3, 3] - x1[:3, 3]
        d_rotvec = R.from_matrix(x2[:3, :3] @ x1[:3, :3].T).as_rotvec(degrees=True)
        return np.concatenate([d_translation, d_rotvec])
