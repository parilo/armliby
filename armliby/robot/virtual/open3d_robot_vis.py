from typing import List, Optional

import numpy as np
import open3d as o3d
from armliby.ik import Kinematics
from armliby.urdf_parser import URDFParser


class Open3dRobotVis:
    def __init__(
            self, 
            urdf_path: str,
            kinematics: Kinematics,
            end_link_name: str,
            skip_links: Optional[List[int]] = None,
            skip_joints: Optional[List[int]] = None,
            ):
        """
        Initialize the virtual robot for visualization.

        Args:
            urdf_path (str): Path to the URDF file.
            joint_count (int): Number of joints in the robot.
        """
        self.urdf_path = urdf_path
        self.parser = URDFParser(
            urdf_path=urdf_path,
            skip_links=skip_links,
            skip_joints=skip_joints,
        )
        self.link_stl_map = self.parser.get_link_stl_map()
        self.kinematics = kinematics
        self.end_link_name = end_link_name
        self.end_link_frame = None
        self.end_link_frame_tr = np.eye(4)
        self.visualizer = o3d.visualization.Visualizer()
        self.geometries = {}
        self.current_transformations = {}
        self.inited = False

    def run(self) -> None:
        """Initialize the Open3D visualizer and load geometries."""
        if self.inited:
            print("Visualizer is already initialized.")
            return

        self.visualizer.create_window()

        for link_name, stl_path in self.link_stl_map.items():
            try:
                mesh = o3d.io.read_triangle_mesh(stl_path)
                if mesh.is_empty():
                    print(f"Warning: STL file {stl_path} is empty.")
                    continue
                mesh.compute_vertex_normals()
                self.visualizer.add_geometry(mesh)
                self.geometries[link_name] = mesh
                self.current_transformations[link_name] = np.eye(4)

            except Exception as e:
                print(f"Error loading STL {stl_path}: {e}")

        # Create a coordinate frame for the link
        frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.2)
        self.visualizer.add_geometry(frame)

        # Create a coordinate frame for the end link
        self.end_link_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        self.visualizer.add_geometry(self.end_link_frame)

        self.inited = True

    def close(self) -> None:
        """Close the Open3D visualizer."""
        if not self.inited:
            print("Visualizer is not initialized.")
            return

        self.visualizer.destroy_window()
        self.geometries.clear()
        self.current_transformations.clear()
        self.inited = False

    def visualize(self, jpos: np.ndarray) -> None:
        """Move the robot to the specified target position."""
        if not self.inited:
            raise RuntimeError("Visualizer is not initialized.")

        # Compute forward kinematics
        fk_results = self.kinematics.fk(jpos)

        # Update visualization
        for link_name, mesh in self.geometries.items():
            if link_name in fk_results:
                transform = fk_results[link_name]

                # Undo the current transformation
                current_transform = self.current_transformations[link_name]

                # Apply the new transformation
                mesh.transform(transform @ np.linalg.inv(current_transform))

                # Store the new transformation
                self.current_transformations[link_name] = transform
                if link_name == self.end_link_name:
                    self.end_link_frame.transform(transform @ np.linalg.inv(self.end_link_frame_tr))
                    self.end_link_frame_tr = transform
                    self.visualizer.update_geometry(self.end_link_frame)

                # Update the geometry in the visualizer
                self.visualizer.update_geometry(mesh)

        self.visualizer.poll_events()
        self.visualizer.update_renderer()
