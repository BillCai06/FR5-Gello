import time
from typing import Any, Dict, Optional

import numpy as np

from gello.cameras.camera import CameraDriver
from gello.robots.robot import Robot


class Rate:
    def __init__(self, rate: float):
        self.last = time.time()
        self.rate = rate

    def sleep(self) -> None:
        while self.last + 1.0 / self.rate > time.time():
            time.sleep(0.0001)
        self.last = time.time()


class RobotEnv:
    def __init__(
        self,
        robot: Robot,
        control_rate_hz: float = 200.0,
        camera_dict: Optional[Dict[str, CameraDriver]] = None,
    ) -> None:
        self._robot = robot
        self._rate = Rate(control_rate_hz)
        self._camera_dict = {} if camera_dict is None else camera_dict

    def robot(self) -> Robot:
        return self._robot

    def step(self, joints: np.ndarray) -> Dict[str, Any]:
        assert len(joints) == self._robot.num_dofs()
        self._robot.command_joint_state(joints)
        # self._robot.command_gripper_state(joints)
        self._rate.sleep()
        return self.get_obs()

    def get_obs(self) -> Dict[str, Any]:
        observations: Dict[str, Any] = {}
        # camera inputs (if any)
        for name, camera in self._camera_dict.items():
            image, depth = camera.read()
            observations[f"{name}_rgb"] = image
            observations[f"{name}_depth"] = depth

        # robot state
        robot_obs = self._robot.get_observations()
        assert "joint_positions" in robot_obs
        assert "joint_velocities" in robot_obs
        assert "ee_pos_quat" in robot_obs

        observations["joint_positions"]  = np.array(robot_obs["joint_positions"])
        observations["joint_velocities"] = robot_obs["joint_velocities"]
        observations["ee_pos_quat"]      = robot_obs["ee_pos_quat"]
        observations["gripper_position"] = robot_obs["gripper_position"]
        return observations


def main() -> None:
    pass


if __name__ == "__main__":
    main()
