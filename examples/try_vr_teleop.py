import os
import time
from typing import Dict

import numpy as np
import open3d as o3d  # need to load open3d before pytorch
from armliby.ik import Kinematics
from armliby.robot.joint_limits import JointLimits
from armliby.robot.virtual.open3d_robot_vis import Open3dRobotVis
from armliby.robot.virtual.virtual_pos_robot import VirtualPosRobot
from armliby.vrteleop.ik_ws_server import ControllerData, VRWebsocketServer
from armliby.vrteleop.vr_teleop_server import VRTeleopServer


SCRIPT_FOLDER = os.path.dirname(__file__)

HOST = '192.168.1.112'
WSS_PORT = 8765
APP_PORT = 5000

URDF_PATH = os.path.realpath(os.path.join(SCRIPT_FOLDER, '../assets/SO_5DOF_ARM100_8j_URDF.SLDASM/SO_5DOF_ARM100_8j_URDF.SLDASM.urdf'))
END_LINK_NAME = "Fixed_Jaw"
VIS_END_LINK_NAME = "Moving Jaw"
CONTROL_FREQ = 25

SSL_CERT = os.path.join(SCRIPT_FOLDER, 'cert.pem')
SSL_KEY = os.path.join(SCRIPT_FOLDER, 'key.pem')

# RELAX_POS = np.array([0., 195, 182, 72.6855, 0])
START_POS = np.array([0., 143, 129, 72.6855, 0, 0])


def main():

    np.set_printoptions(suppress=True, precision=4)

    # Initialize and start the VRWebsocketServer
    # This server is used to send VR controllers data to this script
    ws_server = VRWebsocketServer(
        host=HOST,
        port=WSS_PORT,
        cert_path=SSL_CERT,
        key_path=SSL_KEY,
    )
    ws_server.start()

    # Initialize and start the VRTeleop server
    # This server is used to host web page opened by VR headset
    server = VRTeleopServer(
        host=HOST,
        wss_port=WSS_PORT,
        app_port=APP_PORT,
        urdf_path=URDF_PATH,
        ssl_cert=SSL_CERT,
        ssl_key=SSL_KEY
    )
    server.start()

    # give freedom to rotation over z
    # because we have only 5 DoF
    # and cannot control all 6 DoF
    mod_matrix=np.eye(6)[:5]

    kinematics = Kinematics(
        urdf_path=URDF_PATH,
        end_link_name=END_LINK_NAME,
        mod_matrix=mod_matrix,
    )

    # visualize the robot
    vis_robot = Open3dRobotVis(
        urdf_path=URDF_PATH,
        kinematics=Kinematics(
            urdf_path=URDF_PATH,
            end_link_name=VIS_END_LINK_NAME
        ),
        end_link_name=END_LINK_NAME,
        skip_joints=[0],
    )

    joint_limits = JointLimits.from_urdf(
        urdf_path=URDF_PATH,
        skip_joints=[0],
    )

    # create a virtual robot that will be controlled
    robot = VirtualPosRobot(
        start_joints=START_POS,
        joint_limits=joint_limits,
    )

    # Connect to the robot
    robot.connect()
    vis_robot.run()

    prev_controller_data: ControllerData = None

    def teleop_callback(controller_data: ControllerData) -> Dict[str, np.ndarray]:
        nonlocal prev_controller_data

        if prev_controller_data is not None:
            if (
                len(controller_data.rightController.buttons) > 4 and
                controller_data.rightController.buttons[5].pressed  # B pressed
            ):
                # print current controller pose
                print(controller_data.rightController.pose)

                # calculate the difference between current and previous controller pose
                diff_pose = prev_controller_data.rightController.pose.diff_to(controller_data.rightController.pose)
                rotvec = diff_pose.rotvec()  # in radians

                # calculate the cartesian delta to move the robot
                dx = np.array([
                    diff_pose.x,
                    diff_pose.y,
                    diff_pose.z,
                    rotvec.x,
                    rotvec.y,
                    rotvec.z,
                ])

                cur_joints = robot.read().pos

                # calculate the joint deltas to achieve the cartesian delta
                djoints = kinematics.dj(
                    js=cur_joints[:5],
                    dx=dx,
                )

                # Warning: when use with real robot additional safety is needed
                # beacuse inverse kinematics is unstable near singularities
                # it can provide big djoints values,so robot
                # can move fast and damage itself or environment
                # put additional safety here
                safety_djoints = np.clip(djoints, -30, 30)

                print(f'--- dj {djoints}')

                # update the joint positions
                cur_joints[:5] += safety_djoints
                cur_joints[ 5] = 45 * (1 - controller_data.rightController.buttons[0].value)
                robot.position_abs_control(cur_joints)

                # visualize the robot
                vis_robot.visualize(robot.read().pos)

        prev_controller_data = controller_data
        time.sleep(1. / CONTROL_FREQ)

        # send updated robot links poses to the VR headset
        transforms = kinematics.fk(robot.read().pos[:5])
        return transforms


    try:
        while True:
            ws_server.check(teleop_callback)
    finally:
        robot.relax()
        robot.disconnect()
        server.stop()
        ws_server.stop()


# Start the server
if __name__ == '__main__':
    main()
