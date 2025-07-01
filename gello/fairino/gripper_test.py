import Robot

import time

# Establish a connection with the robot controller and return a robot object if the connection is successful

robot = Robot.RPC('192.168.58.4')

desc_pos1=[-333.683,-228.968,404.329,-179.138,-0.781,91.261]

desc_pos2=[-333.683,-100.8,404.329,-179.138,-0.781,91.261]

zlength1 =10

zlength2 =15

zangle1 =10

zangle2 =15

# Test peripheral commands

ret = robot.SetGripperConfig(4,0) #Configure the gripper jaws

print("Configuration jaw error code", ret)

time.sleep(1)

config = robot.GetGripperConfig() #Get the gripper configuration

print("Getting the jaw configuration",config)

error = robot.ActGripper(1,0) #Activate gripper

print("Activating jaws error code",error)

time.sleep(1)

error = robot.ActGripper(1,1)#Activate gripper

print("Activating jaws error code",error)

time.sleep(2)

error = robot.MoveGripper(1,100,48,46,30000,0,0,0,0,0,0) #Control jaws

print("Control jaw error code",error)

time.sleep(3)

error = robot.MoveGripper(1,0,50,0,30000,0,0,0,0,0,0) #Control jaws

print("Control jaw error code",error)

error = robot.GetGripperMotionDone() #Get Gripper Motion Done

print("Error code for obtaining jaw movement status",error)

error = robot.ComputePrePick(desc_pos1, zlength1, zangle1) #compute pre-pick point-vision

print("Calculating pre-capture points",error)

error = robot.ComputePrePick(desc_pos2, zlength2, zangle2) #calculate retreat point-vision

print("Calculating retreat point",error)