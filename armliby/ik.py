from typing import Dict, Optional
from scipy.spatial.transform import Rotation as R

import numpy as np
import pytorch_kinematics as pk


class Kinematics:
    def __init__(
            self,
            urdf_path: str,
            end_link_name: str,
            mod_matrix: Optional[np.ndarray] = None,
            deg: bool = True,
            ) -> None:
        # Load robot description from URDF and specify end effector link
        with open(urdf_path, "rb") as f:
            urdf_content = f.read()

        self._mod_matrix = mod_matrix
        self._deg = deg

        self.end_link_name = end_link_name
        self._chain = pk.build_serial_chain_from_urdf(
            data=urdf_content,
            end_link_name=end_link_name,
        )

    @property
    def num_dof(self) -> int:
        return len(self._chain.get_joints())

    def fk(self, js: np.ndarray) -> Dict[str, np.ndarray]:
        # Perform forward kinematics and get transform objects
        if self._deg:
            js = np.deg2rad(js)

        ret = self._chain.forward_kinematics(js, end_only=False)
        ret = {key: val.get_matrix().cpu().detach().numpy()[0] for key, val in ret.items()}
        return ret

    def dj(
            self,
            js: np.ndarray,
            dx: np.ndarray,
            ) -> np.ndarray:

        if self._deg:
            js = np.deg2rad(js)
            dx = dx.copy()
            dx[3:] = np.deg2rad(dx[3:])

        jac = self._chain.jacobian(js).detach().numpy()[0]
        if self._mod_matrix is not None:
            jac = self._mod_matrix @ jac
            dx = self._mod_matrix @ dx

        jac_inv = np.linalg.pinv(jac)
        dj = jac_inv @ dx

        if self._deg:
            dj = np.rad2deg(dj)

        return dj

    def dx(
            self,
            js1: np.ndarray,
            js2: np.ndarray,
            ) -> np.ndarray:
        
        x1 = self.fk(js1)[self.end_link_name]
        x2 = self.fk(js2)[self.end_link_name]
        d_translation = x2[:3, 3] - x1[:3, 3]
        d_rotvec = R.from_matrix(x2[:3, :3] @ x1[:3, :3].T).as_rotvec(degrees=True)
        return np.concatenate([d_translation, d_rotvec])
