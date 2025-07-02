import os
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

import numpy as np

from gello.agents.agent import Agent
from gello.robots.dynamixel import DynamixelRobot


@dataclass
class DynamixelRobotConfig:
    joint_ids: Sequence[int]
    """The joint ids of GELLO (not including the gripper). Usually (1, 2, 3 ...)."""

    joint_offsets: Sequence[float]
    """The joint offsets of GELLO. There needs to be a joint offset for each joint_id and should be a multiple of pi/2."""

    joint_signs: Sequence[int]
    """The joint signs of GELLO. There needs to be a joint sign for each joint_id and should be either 1 or -1.

    This will be different for each arm design. Refernce the examples below for the correct signs for your robot.
    """

    gripper_config: Tuple[int, int, int]
    """The gripper config of GELLO. This is a tuple of (gripper_joint_id, degrees in open_position, degrees in closed_position)."""

    def __post_init__(self):
        assert len(self.joint_ids) == len(self.joint_offsets)
        assert len(self.joint_ids) == len(self.joint_signs)

    def make_robot(
        self, port: str = "/dev/ttyUSB0", start_joints: Optional[np.ndarray] = None
    ) -> DynamixelRobot:
        return DynamixelRobot(
            joint_ids=self.joint_ids,
            joint_offsets=list(self.joint_offsets),
            real=True,
            joint_signs=list(self.joint_signs),
            port=port,
            gripper_config=self.gripper_config,
            start_joints=start_joints,
        )


PORT_CONFIG_MAP: Dict[str, DynamixelRobotConfig] = {
    # FR5
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NLPE-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            3.5 * np.pi / 2,   # 4.712
            2 * np.pi / 2,   # 3.142
            0 * np.pi / 2,   # -3.142
            5 * np.pi / 2,   # 1.571
            2 * np.pi / 2,   # 3.142
            3 * np.pi / 2,   # 4.712
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 21.508984375, -20.291015625),
    ),

    # panda
    # "/dev/cu.usbserial-FT3M9NVB": DynamixelRobotConfig(
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT3M9NVB-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6, 7),
        joint_offsets=(
            3 * np.pi / 2,
            2 * np.pi / 2,
            1 * np.pi / 2,
            4 * np.pi / 2,
            -2 * np.pi / 2 + 2 * np.pi,
            3 * np.pi / 2,
            4 * np.pi / 2,
        ),
        joint_signs=(1, -1, 1, 1, 1, -1, 1),
        gripper_config=(8, 195, 152),
    ),
    # one_UR_only
    # one_UR_only
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NNT1-if00-port0": DynamixelRobotConfig(
    joint_ids=(1, 2, 3, 4, 5, 6),
    joint_offsets=(
        0 * np.pi / 2,  # 0.000
        3 * np.pi / 2,  # 4.712
        6 * np.pi / 2,  # 9.425
        4 * np.pi / 2,  # 6.283
        2 * np.pi / 2,  # 3.142
        -4 * np.pi / 2,  # -6.283
    ),
    joint_signs=(1, 1, -1, 1, 1, 1),
    gripper_config=(7, 291.860546875, 250.060546875),
),
    
    
    # Left UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBEIA-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            0,
            1 * np.pi / 2 + np.pi,
            np.pi / 2 + 0 * np.pi,
            0 * np.pi + np.pi / 2,
            np.pi - 2 * np.pi / 2,
            -1 * np.pi / 2 + 2 * np.pi,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 20, -22),
    ),
    # Right UR
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT7WBG6A-if00-port0": DynamixelRobotConfig(
        joint_ids=(1, 2, 3, 4, 5, 6),
        joint_offsets=(
            np.pi + 0 * np.pi,
            2 * np.pi + np.pi / 2,
            2 * np.pi + np.pi / 2,
            2 * np.pi + np.pi / 2,
            1 * np.pi,
            3 * np.pi / 2,
        ),
        joint_signs=(1, 1, -1, 1, 1, 1),
        gripper_config=(7, 286, 248),
    ),
}


class GelloAgent(Agent):
    def __init__(
        self,
        port: str,
        dynamixel_config: Optional[DynamixelRobotConfig] = None,
        start_joints: Optional[np.ndarray] = None,
    ):
        if dynamixel_config is not None:
            self._robot = dynamixel_config.make_robot(
                port=port, start_joints=start_joints
            )
        else:
            assert os.path.exists(port), port
            assert port in PORT_CONFIG_MAP, f"Port {port} not in config map"

            config = PORT_CONFIG_MAP[port]
            self._robot = config.make_robot(port=port, start_joints=start_joints)
    #Where to set the gripper pos pub 
    def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
        full_joint_state = self._robot.get_joint_state()  # length 7
        arm_joint_state = full_joint_state[:-1]           # length 6 (removes gripper)
        return full_joint_state


    # def act(self, obs: Dict[str, np.ndarray]) -> np.ndarray:
    #     return self._robot.get_joint_state().shape
    #     dyna_joints = self._robot.get_joint_state()
    #     # current_q = dyna_joints[:-1]  # last one dim is the gripper
    #     current_gripper = dyna_joints[-1]  # last one dim is the gripper

    #     print(current_gripper)
    #     if current_gripper < 0.2:
    #         self._robot.set_torque_mode(False)
    #         return obs["joint_positions"]
    #     else:
    #         self._robot.set_torque_mode(False)
    #         return dyna_joints