import PX4MavCtrlV4 as PX4MavCtrl
import VisionCaptureApi
import UE4CtrlAPI
import ReqCopterSim

import sys

Local_IP = "192.168.137.1"  # local ip of windows
Remote_IP = "192.168.137.2"  # remote ip of Ubuntu
VehilceNum = 1
StartCopterID = 1
req = ReqCopterSim.ReqCopterSim()
for i in range(VehilceNum):
    req.sendReSimUdpMode(StartCopterID + i, 2)  # Mavlink_full mode

# sensor
vis = VisionCaptureApi.VisionCaptureApi()
vis.jsonLoad(jsonPath='./config/Config_ROS.json') # create sensor
ue = UE4CtrlAPI.UE4CtrlAPI()

vis.isUE4DirectUDP=True

isSuss = vis.sendReqToUE4()  # send request to Rflysim3D
if not isSuss:  
    print('The request for image capture failed.')
    sys.exit(0)
vis.startImgCap()  # send img

vis.RemotSendIP = Remote_IP
vis.sendImuReqCopterSim(copterID=1, IP=Local_IP) # send imu


################### motion ################################
import time
import numpy as np

# trajectory params
A = 10.0          # radius of 8
height = 20.0     # flight height
desired_speed = 0.5 # flight velocity
takeoff_speed = 0.5
dt = 0.05        # control period
T_total = 50    # total time


MavList = []
# Create MAV instance
for i in range(VehilceNum):
    CopterID = StartCopterID + i # idx of this uav
    TargetIP = req.getSimIpID(CopterID) # IP of uav
    req.sendReSimIP(CopterID)
    time.sleep(1)
    MavList = MavList + [PX4MavCtrl.PX4MavCtrler(CopterID, TargetIP)] 

time.sleep(2)
# # Start MAV loop with UDP mode: MAVLINK_FULL
for i in range(VehilceNum):
    MavList[i].InitMavLoop()
    MavList[i].InitTrueDataLoop()

# # Enter Offboard mode to start vehicle control
time.sleep(2)
for i in range(VehilceNum):
    MavList[i].initOffboard()

    print("taking off...")

# 8 trajectory
for i in range(VehilceNum):
    MavList[i].SendVelNED(0, 0, -takeoff_speed) 
    time.sleep(height / takeoff_speed)
    
    print("start to fly along 8...")
    t = 0
    x0 = MavList[i].uavPosNED[0]
    y0 = MavList[i].uavPosNED[1]
    while t < T_total:
        dx_dt = -A * np.sin(t)
        dy_dt = A * np.cos(2 * t)
        
        inst_speed = np.sqrt(dx_dt**2 + dy_dt**2)
        
        target_x = A * np.cos(t) + x0 - A
        target_y = A * np.sin(t) * np.cos(t) + y0
        
        MavList[i].SendPosNED(target_x, target_y, -height, 0)
        
        dt_param = (desired_speed / inst_speed) * dt
        t += dt_param
        
        time.sleep(dt)