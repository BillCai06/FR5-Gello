"""
FR5 Robot Interface for Gello Teleoperation System
This module provides the robot interface to connect FR5 robotic arm with Gello teleop system.
"""

import numpy as np
import time
import array  # ← Add this line
import threading
from typing import Dict, List, Optional, Tuple
import logging
from dataclasses import dataclass

# Import your existing RPC class
from gello.fairino.Robot import RPC, RobotStatePkg, RobotError

# Gello Robot Protocol Interface
class Robot:
    """Robot protocol interface that FR5Robot must implement"""
    
    def get_joint_state(self) -> np.ndarray:
        """Get current joint positions in radians"""
        raise NotImplementedError
    
    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Send joint position commands in radians"""
        raise NotImplementedError
    
    # def get_gripper_state(self) -> float:
    #     """Get current gripper position (0.0 = closed, 1.0 = open)"""
    #     raise NotImplementedError
    
    # def command_gripper_state(self, gripper_state: float) -> None:
    #     """Command gripper position (0.0 = closed, 1.0 = open)"""
    #     raise NotImplementedError


@dataclass
class FR5Config:
    """Configuration for FR5 robot parameters"""
    ip_address: str = "192.168.58.2"
    joint_count: int = 6
    joint_limits_lower: List[float] = None  # in radians
    joint_limits_upper: List[float] = None  # in radians
    max_joint_velocity: float = 1.0  # rad/s
    max_joint_acceleration: float = 2.0  # rad/s^2
    control_frequency: float = 100.0  # Hz
    
    def __post_init__(self):
        if self.joint_limits_lower is None:
            # Default FR5 joint limits in radians (adjust based on your robot specs)
            self.joint_limits_lower = [-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi]
        if self.joint_limits_upper is None:
            self.joint_limits_upper = [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]


class FR5Robot(Robot):
    """FR5 Robot interface for Gello teleoperation"""
    
    def __init__(self, config: FR5Config = None):
        self.config = config or FR5Config()
        self.rpc_client = None
        self.is_connected = False
        self.is_enabled = False
        self._use_gripper = False  # Assuming FR5 has a gripper
        
        # Joint state tracking
        self.current_joint_positions = np.zeros(self.config.joint_count)
        self.target_joint_positions = np.zeros(self.config.joint_count)
        self.last_command_time = time.time()
        
        # Gripper state
        self.current_gripper_position = 1000000.0
        self.target_gripper_position = 0.0
        
        # Safety and control
        self.emergency_stop = False
        self.joint_limits_lower = np.array(self.config.joint_limits_lower)
        self.joint_limits_upper = np.array(self.config.joint_limits_upper)
        
        # Logging
        self.logger = logging.getLogger(__name__)
        
        # Connect to robot
        self.connect()
    
    def connect(self) -> bool:
        """Connect to FR5 robot"""
        try:
            self.logger.info(f"Connecting to FR5 robot at {self.config.ip_address}")
            self.rpc_client = RPC(ip=self.config.ip_address)
            
            if not RPC.is_conect:
                self.logger.error("Failed to connect to FR5 robot")
                return False
            
            self.is_connected = True
            self.logger.info("Successfully connected to FR5 robot")
            
            # Initialize robot to position control mode
            self.enable_robot()
            
            # Get initial joint state
            self.update_joint_state()
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error connecting to FR5: {e}")
            return False
    
    def enable_robot(self) -> bool:
        """Enable robot for operation"""
        try:
            if not self.is_connected:
                self.logger.error("Robot not connected")
                return False
            
            # Enable robot (assuming Mode(1) enables the robot)
            result = self.rpc_client.robot.Mode(1)
            if result != 0:
                self.logger.error(f"Failed to enable robot, error code: {result}")
                return False
            
            time.sleep(0.1)  # Allow time for mode change
            
            # Set to position control mode (adjust based on your robot's API)
            # This is a placeholder - adjust based on your actual robot API
            # result = self.rpc_client.robot.SetControlMode(0)  # 0 for position control
            
            self.is_enabled = True
            self.logger.info("Robot enabled successfully")
            return True
            
        except Exception as e:
            self.logger.error(f"Error enabling robot: {e}")
            return False
    
    def disable_robot(self) -> bool:
        """Disable robot"""
        try:
            if self.rpc_client:
                result = self.rpc_client.robot.Mode(0)
                self.is_enabled = False
                self.logger.info("Robot disabled")
                return result == 0
        except Exception as e:
            self.logger.error(f"Error disabling robot: {e}")
            return False
    
    def update_joint_state(self) -> bool:
        """Update current joint positions from robot state"""
        try:
            if not self.is_connected or not self.rpc_client:
                return False
            
            # Get robot state from the real-time data stream
            robot_state = self.rpc_client.robot_state_pkg
            
            if robot_state:
                # Extract joint positions (assuming jt_cur_pos contains joint positions)
                # Convert from degrees to radians if necessary
                joint_positions = np.array(robot_state.jt_cur_pos[:self.config.joint_count])
                
                # Check if the data is valid (your SDK sets positions to 0 on checksum error)
                if not np.allclose(joint_positions, 0.0):
                    self.current_joint_positions = np.deg2rad(joint_positions)  # Convert to radians
                
                return True
            
        except Exception as e:
            self.logger.error(f"Error updating joint state: {e}")
            return False
    #  get_observations
    def get_joint_state(self) -> np.ndarray:
        """Get current joint positions in radians"""
        self.update_joint_state()
        return self.current_joint_positions.copy()
    
    def get_observations(self) -> Dict[str, np.ndarray]:
        joints = self.current_joint_positions.copy()
        pos_quat = np.zeros(7)  # Placeholder for end-effector position and orientation
        gripper_pos = np.array([joints[-1]])
        return {
            "joint_positions": joints,
            "joint_velocities": joints,
            "ee_pos_quat": pos_quat,
            "gripper_position": gripper_pos,
        }
    
    
    def num_dofs(self) -> int:
        """Get the number of joints of the robot.

        Returns:
            int: The number of joints of the robot.
        """
        if self._use_gripper:
            # ue:print("gripper detected")
            return 7
        return 6


    def command_joint_state(self, joint_state: np.ndarray) -> None:
        """Send joint position commands in radians"""
        try:
            # if not self.is_connected or not self.is_enabled:
            #     self.logger.warning("Robot not connected or enabled")
            #     return
            
            if self.emergency_stop:
                self.logger.warning("Emergency stop active")
                return
            
            # Validate joint limits
            joint_state_clamped = np.clip(joint_state, 
                                        self.joint_limits_lower, 
                                        self.joint_limits_upper)
            
            if not np.allclose(joint_state, joint_state_clamped):
                self.logger.warning("Joint command clamped to limits")
            
            # Convert from radians to degrees for the robot API
            joint_degrees = np.rad2deg(joint_state_clamped)
            # *** APPLY YOUR -45° OFFSET ON JOINT 0 ***
            # joint_degrees[0] -= 45.0


            if hasattr(self.rpc_client.robot, 'MoveJ'):
                joint_list = joint_degrees.tolist()
                
             
                joint_array = array.array('d', joint_list)
                try:
                    self.rpc_client.robot.Stop()
                except Exception:
                    pass  # if Stop() isn’t available or fails, we ignore

                tool = 0
                user = 0
                print(f"-----------------------------------------------------") 
                print(f"Commanding joint state: {joint_array}") 
               
                result = self.rpc_client.MoveJ(joint_array, tool, user)
                if result != 0:
                    self.logger.error(f"Joint command failed with error: {result}")

            
            self.target_joint_positions = joint_state_clamped.copy()
            self.last_command_time = time.time()
            
        except Exception as e:
            self.logger.error(f"Error commanding joint state: {e}")
    
    #  def get_gripper_state(self) -> float:
    #     """
    #     Get current gripper motion status.
    #     Returns:
    #         1.0 if motion is complete (jaw in position),
    #         0.0 if still moving or on error.
    #     """
    #     try:
    #         # RPC GetGripperMotionDone returns (error_code, [fault, status])
    #         ret = self.rpc_client.GetGripperMotionDone()
    #         if not isinstance(ret, tuple) or len(ret) != 2:
    #             self.logger.error(f"Unexpected GetGripperMotionDone reply: {ret}")
    #             return 0.0

    #         error, flags = ret
    #         if error != 0:
    #             self.logger.error(f"GetGripperMotionDone error: {error}")
    #             return 0.0

    #         fault, status = flags  # status: 0 = moving, 1 = done
    #         if fault != 0:
    #             self.logger.warning(f"Gripper fault detected: {fault}")

    #         return float(status)

    #     except Exception as e:
    #         self.logger.error(f"Error getting gripper state: {e}")
    #         return 0.0


    def command_gripper_state(self, gripper_state: float) -> None:
        """
        Command gripper to a position.
        gripper_state: 0.0 = fully closed, 1.0 = fully open
        """
        try:
            if not self.is_connected or not self.is_enabled:
                self.logger.warning("Robot not connected or enabled")
                return

            # Clamp input
            cmd = float(np.clip(gripper_state, 0.0, 1.0))

            # Map to percentage [0–100]
            pos_pct = int(cmd * 100.0)

            # Defaults for speed, force, time, blocking, type, rotation args:
            vel_pct    = 50    # 50% speed
            force_pct  = 50    # 50% torque
            maxtime_ms = 30000 # wait up to 30 s
            block      = 0     # non-blocking
            gtype      = 0     # parallel gripper
            rotNum     = 0.0
            rotVel_pct = 0
            rotTor_pct = 0

            # Flush any pending motion so we jump immediately
            try:
                self.rpc_client.robot.Stop()
            except Exception:
                pass

            # Send the MoveGripper call
            error = self.rpc_client.MoveGripper(
                index     = 1,        # your single gripper index
                pos       = pos_pct,
                vel       = vel_pct,
                force     = force_pct,
                maxtime   = maxtime_ms,
                block     = block,
                type      = gtype,
                rotNum    = rotNum,
                rotVel    = rotVel_pct,
                rotTorque = rotTor_pct
            )

            if error != 0:
                self.logger.error(f"MoveGripper failed with code: {error}")

            # Record target as normalized [0–1]
            self.target_gripper_position = cmd

        except Exception as e:
            self.logger.error(f"Error commanding gripper state: {e}")
    
    def emergency_stop_activate(self) -> None:
        """Activate emergency stop"""
        self.emergency_stop = True
        self.logger.warning("Emergency stop activated")
        
        # Stop robot motion
        try:
            if self.rpc_client and hasattr(self.rpc_client.robot, 'Stop'):
                self.rpc_client.robot.Stop()
        except Exception as e:
            self.logger.error(f"Error during emergency stop: {e}")
    
    def emergency_stop_reset(self) -> None:
        """Reset emergency stop"""
        self.emergency_stop = False
        self.logger.info("Emergency stop reset")
    
    def get_robot_status(self) -> Dict:
        """Get comprehensive robot status"""
        return {
            'connected': self.is_connected,
            'enabled': self.is_enabled,
            'emergency_stop': self.emergency_stop,
            'joint_positions': self.current_joint_positions.tolist(),
            'target_joint_positions': self.target_joint_positions.tolist(),
            'gripper_position': self.current_gripper_position,
            'target_gripper_position': self.target_gripper_position,
            'last_command_time': self.last_command_time,
            'sdk_connection_state': RPC.is_conect if self.rpc_client else False
        }
    
    def __del__(self):
        """Cleanup when object is destroyed"""
        self.cleanup()
    
    def cleanup(self) -> None:
        """Clean shutdown of robot connection"""
        try:
            self.disable_robot()
            if self.rpc_client:
                # Close RPC connection
                self.rpc_client.closeRPC_state = True
                self.rpc_client = None
            self.is_connected = False
            self.logger.info("FR5 robot connection cleaned up")
        except Exception as e:
            self.logger.error(f"Error during cleanup: {e}")


# Gello Integration Helper Functions
def create_fr5_robot_config() -> Dict:
    """Create FR5 robot configuration for Gello system"""
    return {
        'robot_type': 'fr5',
        'joint_count': 6,
        'has_gripper': False,
        'control_frequency': 100.0,
        'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
        'joint_limits': {
            'lower': [-np.pi, -np.pi, -np.pi, -np.pi, -np.pi, -np.pi],
            'upper': [np.pi, np.pi, np.pi, np.pi, np.pi, np.pi]
        }
    }


def main():
    """Test the FR5 robot interface"""
    logging.basicConfig(level=logging.INFO)
    
    # Create robot instance
    config = FR5Config(ip_address="192.168.58.2")
    robot = FR5Robot(config)
    
    if not robot.is_connected:
        print("Failed to connect to robot")
        return
    
    print("Robot connected successfully!")
    
    try:
        # Test getting joint state
        joint_state = robot.get_joint_state()
        print(f"Current joint positions: {joint_state}")
        
        # Test robot status
        status = robot.get_robot_status()
        print(f"Robot status: {status}")
        # Keep the connection alive for testing
        for i in range(10):
            joint_state = robot.get_joint_state()
            # gripper_state = robot.get_gripper_state()
            print(f"Joints{i}: {joint_state}, Gripper: NILL")
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        robot.cleanup()


if __name__ == "__main__":
    main()