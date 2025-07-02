import datetime
import glob
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np
import tyro

from gello.agents.agent import BimanualAgent, DummyAgent
from gello.agents.gello_agent import GelloAgent
from gello.data_utils.format_obs import save_frame
from gello.env import RobotEnv
from gello.robots.robot import PrintRobot
from gello.zmq_core.robot_node import ZMQClientRobot


def print_color(*args, color=None, attrs=(), **kwargs):
    import termcolor

    if len(args) > 0:
        args = tuple(termcolor.colored(arg, color=color, attrs=attrs) for arg in args)
    print(*args, **kwargs)


@dataclass
class Args:
    agent: str = "none"
    robot_port: int = 6001
    wrist_camera_port: int = 5000
    base_camera_port: int = 5001
    hostname: str = "127.0.0.1"
    robot_type: str = None  # only needed for quest agent or spacemouse agent
    hz: int = 200
    start_joints: Optional[Tuple[float, ...]] = None

    gello_port: Optional[str] = None
    mock: bool = False
    use_save_interface: bool = False
    data_dir: str = "~/bc_data"
    bimanual: bool = False
    verbose: bool = False


def main(args):
    if args.mock:
        robot_client = PrintRobot(8, dont_print=True)
        camera_clients = {}
    else:
        camera_clients = {
            # you can optionally add camera nodes here for imitation learning purposes
            # "wrist": ZMQClientCamera(port=args.wrist_camera_port, host=args.hostname),
            # "base": ZMQClientCamera(port=args.base_camera_port, host=args.hostname),
        }
        robot_client = ZMQClientRobot(port=args.robot_port, host=args.hostname)
    env = RobotEnv(robot_client, control_rate_hz=args.hz, camera_dict=camera_clients)

    if args.bimanual:
        if args.agent == "gello":
            # dynamixel control box port map (to distinguish left and right gello)
            right = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NLPE-if00-port0"
            left = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTA7NLPE-if00-port0"
            left_agent = GelloAgent(port=left)
            right_agent = GelloAgent(port=right)
            agent = BimanualAgent(left_agent, right_agent)
        elif args.agent == "quest":
            from gello.agents.quest_agent import SingleArmQuestAgent

            left_agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="l")
            right_agent = SingleArmQuestAgent(
                robot_type=args.robot_type, which_hand="r"
            )
            agent = BimanualAgent(left_agent, right_agent)
            # raise NotImplementedError
        elif args.agent == "spacemouse":
            from gello.agents.spacemouse_agent import SpacemouseAgent

            left_path = "/dev/hidraw0"
            right_path = "/dev/hidraw1"
            left_agent = SpacemouseAgent(
                robot_type=args.robot_type, device_path=left_path, verbose=args.verbose
            )
            right_agent = SpacemouseAgent(
                robot_type=args.robot_type,
                device_path=right_path,
                verbose=args.verbose,
                invert_button=True,
            )
            agent = BimanualAgent(left_agent, right_agent)
        else:
            raise ValueError(f"Invalid agent name for bimanual: {args.agent}")

        # System setup specific. This reset configuration works well on our setup. If you are mounting the robot
        # differently, you need a separate reset joint configuration.
        reset_joints_left = np.deg2rad([0, -90, -90, -90, 90, 0, 0])
        reset_joints_right = np.deg2rad([0, -90, 90, -90, -90, 0, 0])
        reset_joints = np.concatenate([reset_joints_left, reset_joints_right])
        curr_joints = env.get_obs()["joint_positions"]
        max_delta = (np.abs(curr_joints - reset_joints)).max()
        steps = min(int(max_delta / 0.01), 100)

        for jnt in np.linspace(curr_joints, reset_joints, steps):
            env.step(jnt)
    else:
        if args.agent == "gello":
            gello_port = args.gello_port
            if gello_port is None:
                usb_ports = glob.glob("/dev/serial/by-id/*")
                print(f"Found {len(usb_ports)} ports")
                if len(usb_ports) > 0:
                    gello_port = usb_ports[0]
                    print(f"using port {gello_port}")
                else:
                    raise ValueError(
                        "No gello port found, please specify one or plug in gello"
                    )
            if args.start_joints is None:
                reset_joints = np.deg2rad(
                    [0, -90, 90, -90, -90, 0, 0]
                )  # Change this to your own reset joints
            else:
                reset_joints = args.start_joints
            agent = GelloAgent(port=gello_port, start_joints=args.start_joints)
            curr_joints = env.get_obs()["joint_positions"]
            if reset_joints.shape == curr_joints.shape:
                max_delta = (np.abs(curr_joints - reset_joints)).max()
                steps = min(int(max_delta / 0.01), 100)

                for jnt in np.linspace(curr_joints, reset_joints, steps):
                    env.step(jnt)
                    time.sleep(0.001)
        elif args.agent == "quest":
            from gello.agents.quest_agent import SingleArmQuestAgent

            agent = SingleArmQuestAgent(robot_type=args.robot_type, which_hand="l")
        elif args.agent == "spacemouse":
            from gello.agents.spacemouse_agent import SpacemouseAgent

            agent = SpacemouseAgent(robot_type=args.robot_type, verbose=args.verbose)
        elif args.agent == "dummy" or args.agent == "none":
            agent = DummyAgent(num_dofs=robot_client.num_dofs())
        elif args.agent == "policy":
            raise NotImplementedError("add your imitation policy here if there is one")
        else:
            raise ValueError("Invalid agent name")

    # going to start position
    # print("Going to start position")
    # start_pos = agent.act(env.get_obs())
    # obs = env.get_obs()
    # joints = obs["joint_positions"]
    # print(f"starting pos: {start_pos}")
    # print(f"Joints: {Joints}")
    

    print("Going to start position")
    # is the gello agent
    start_pos = agent.act(env.get_obs())
    start_pos = start_pos[:6]
    print(f"Starting pos (rad): {start_pos}")
    # grab the newest observation
    # this is the robot arm
    obs = env.get_obs()
    # joint_positions is assumed to be in radians
    joints = obs["joint_positions"]

    # convert to degrees
    joints_deg = np.degrees(joints)
    start_deg = np.degrees(start_pos[:6])   # assume first 6 joints are the gello arm
    # slice out GELLO vs FR5 (here GELLO is joints 0–5, FR5 is the rest)
    fr5_angles = joints_deg[:6]
    gello_angles = start_deg[:6]

    print(f"Starting pos (rad): {start_pos}")
    print(f"GELLO joint angles (°): {gello_angles.tolist()}")
    print(f"FR5  joint angles (°): {fr5_angles.tolist()}")

    abs_deltas = np.abs(start_pos - joints)
    id_max_joint_delta = np.argmax(abs_deltas)

    max_joint_delta = 1.8
    if abs_deltas[id_max_joint_delta] > max_joint_delta:
        id_mask = abs_deltas > max_joint_delta
        print("error: joint delta too big")
        print(f"Max joint delta: {abs_deltas[id_max_joint_delta]}")
        ids = np.arange(len(id_mask))[id_mask]
        for i, delta, joint, current_j in zip(
            ids,
            abs_deltas[id_mask],
            start_pos[id_mask],
            joints[id_mask],
        ):
            print(
                f"joint[{i}]: \t delta: {delta:4.3f} , leader: \t{joint:4.3f} , follower: \t{current_j:4.3f}"
            )
        return

    print(f"Start pos: {len(start_pos)}", f"Joints: {len(joints)}")
    assert len(start_pos) == len(
        joints
    ), f"agent output dim = {len(start_pos)}, but env dim = {len(joints)}"

    max_delta = 0.1
   # slew into start pose in small increments
    for _ in range(12):
        obs = env.get_obs()
        command_joints = agent.act(obs)
        command_joints = command_joints[:6]  # assume first 6 joints are the gello arm
        current_joints = obs["joint_positions"]
        delta = command_joints - current_joints

        # limit per‐step motion to max_delta radians
        max_step = max_delta
        delta_norm = np.abs(delta).max()
        if delta_norm > max_step:
            delta = (delta / delta_norm) * max_step

        env.step(current_joints + delta)

    # final safety check using the same threshold
    obs = env.get_obs()
    joints = obs["joint_positions"]
    action = agent.act(obs)
    action = action[:6]  # assume first 6 joints are the gello arm
    delta = action - joints

    max_allowed = 1.7  # e.g. 0.6
    mask = np.abs(delta) > max_allowed
    if mask.any():
        print("Final action too large on some joints:")
        bad_idxs = np.where(mask)[0]
        for j in bad_idxs:
            print(
                f" Joint[{j}]: cmd={action[j]:.3f}, "
                f"curr={joints[j]:.3f}, diff={delta[j]:.3f}"
            )
        return  # or exit() if you truly want to quit

    # if we get here, everything is within max_delta
    # print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))


    if args.use_save_interface:
        from gello.data_utils.keyboard_interface import KBReset

        kb_interface = KBReset()

    print_color("\nStart 🚀🚀🚀", color="green", attrs=("bold",))



    save_path = None
    start_time = time.time()

    # fixed-timestep parameters
    interval = 1.0 / args.hz
    next_call = time.monotonic()

    while True:
        # show total elapsed time
        elapsed = time.time() - start_time
        message = f"\rTime passed: {elapsed:.2f} s"
        print_color(message, color="white", attrs=("bold",), end="", flush=True)

        # sense & decide
        obs = env.get_obs()
        action = agent.act(obs)
        dt = datetime.datetime.now()

        # optional saving interface
        if args.use_save_interface:
            state = kb_interface.update()
            if state == "start":
                dt_time = datetime.datetime.now()
                save_path = (
                    Path(args.data_dir).expanduser()
                    / args.agent
                    / dt_time.strftime("%m%d_%H%M%S")
                )
                save_path.mkdir(parents=True, exist_ok=True)
                print(f"\nSaving to {save_path}")
            elif state == "save":
                assert save_path is not None, "something went wrong"
                save_frame(save_path, dt, obs, action)
            elif state == "normal":
                save_path = None
            else:
                raise ValueError(f"Invalid state {state}")

        # act
        obs = env.step(action)

        # throttle to exact timestep
        next_call += interval
        to_sleep = next_call - time.monotonic()
        if to_sleep > 0:
            time.sleep(to_sleep)


if __name__ == "__main__":
    main(tyro.cli(Args))