from gello.fairino.Robot import RPC, RobotStatePkg, RobotError
import time

# Establish a connection with the robot controller and return a robot object if the connection is successful

robot = RPC('192.168.58.2')

joint_pos4 = [45, -90, 90, -90, -90, 0]
joint_posT = [83.84765625000001, -91.494140625, 91.23046875, -86.748046875, -95.361328125, -8.085937499999986]
joint_pos5 = [-43.24, -70.476, 93.688, -114.079, -62, -80]

joint_pos6 = [-83.24, -96.416, 43.188, -74.079, -80, -10]

tool = 0 #Tool coordinate system number

user = 0 #Workpiece coordinate system number

ret = robot.MoveJ(joint_pos4, tool, user) #joint space motion

print("Joint space motion point 4: error code", ret)
ret = robot.SetGripperConfig(6,0) #Configure the gripper jaws
time.sleep(1)
print("Configuration jaw error code", ret)
config = robot.GetGripperConfig() #Get the gripper configuration
time.sleep(1)
print("Getting the jaw configuration",config)
time.sleep(1)
error = robot.ActGripper(1,0) #Activate gripper
time.sleep(1)

error = robot.ActGripper(1,1)#Activate gripper
time.sleep(1)
print("Activating jaws error code",error)

time.sleep(2)

error = robot.MoveGripper(1,100,50,50,30000,0,0,0,0,0) #Control jaws

print("Control jaw error code",error)
error = robot.GetAxleLuaGripperFunc(1)#Get Gripper Motion Done

print("Error code for obtaining jaw movement status",error)
time.sleep(3)