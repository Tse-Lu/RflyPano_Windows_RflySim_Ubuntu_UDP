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

# sensor: only the front-left fisheye camera (UDP port 10002) is forwarded for Kalibr
vis = VisionCaptureApi.VisionCaptureApi()
vis.jsonLoad(jsonPath='./config/Config_Fisheye_FL.json')  # create the front-left fisheye sensor
ue = UE4CtrlAPI.UE4CtrlAPI()

vis.isUE4DirectUDP = True

isSuss = vis.sendReqToUE4()  # send request to Rflysim3D
if not isSuss:
    print('The request for image capture failed.')
    sys.exit(0)
vis.startImgCap()  # forward fisheye image via UDP to Remote_IP

vis.RemotSendIP = Remote_IP
vis.sendImuReqCopterSim(copterID=1, IP=Local_IP)  # forward imu via the same UDP for cam-imu calibration


################### motion ################################
import time
import math
import numpy as np

# flight params
flight_height = 1.5    # take-off / trajectory altitude above the take-off point [m]
takeoff_speed = 0.5    # climb speed [m/s]
dt = 0.05              # control period [s]
T_total = 120.0        # data collection duration [s]

# The front-left fisheye is mounted at body yaw -45 deg, so keep the drone at base
# yaw +45 deg: the camera then looks toward world +X (north), where the board is placed.
base_yaw = np.radians(45.0)

# Kalibr cam-imu excitation: figure-8 (xy) + altitude sinusoid + yaw oscillation
A_xy = 0.2                       # horizontal figure-8 amplitude [m]
A_z = 0.2                        # vertical oscillation amplitude [m]
yaw_amp = np.radians(25.0)       # yaw oscillation amplitude [rad]
w_xy = 0.6                       # figure-8 angular speed [rad/s]
w_z = 0.9                        # vertical angular speed [rad/s]
w_yaw = 0.5                      # yaw angular speed [rad/s]

# checkerboard calibration board (RflySim 12x8 chessboard model, vehicle type 40)
board_id = 100
board_type = 40
board_dist = 0.8                 # placed north of the drone [m]

MavList = []
# Create MAV instance
for i in range(VehilceNum):
    CopterID = StartCopterID + i  # idx of this uav
    TargetIP = req.getSimIpID(CopterID)  # IP of uav
    req.sendReSimIP(CopterID)
    time.sleep(1)
    MavList = MavList + [PX4MavCtrl.PX4MavCtrler(CopterID, TargetIP)]

time.sleep(2)
# # Start MAV loop with UDP mode: MAVLINK_FULL
for i in range(VehilceNum):
    MavList[i].InitMavLoop()
    MavList[i].InitTrueDataLoop()

# wait for the true (UE4 map-center frame) state of the drone, which is the same
# coordinate frame that sendUE4Pos uses to draw objects in RflySim3D
for _ in range(200):
    if MavList[0].trueTimeStmp > 0:
        break
    time.sleep(0.05)

# place the STATIC checkerboard in the RflySim3D scene, relative to the drone's TRUE pose
# (truePosNED is the UE4 map-center frame, NOT the take-off-relative uavPosNED).
# It is placed at the flight altitude so the drone climbs to roughly the board height.
trueX0 = MavList[0].truePosNED[0]
trueY0 = MavList[0].truePosNED[1]
trueZ0 = MavList[0].truePosNED[2]
TargePos = [trueX0 + board_dist, trueY0, trueZ0 - flight_height]  # north, at flight altitude
TargeAng = [math.pi / 2, 0, math.pi / 2]                          # upright, facing south toward the drone
for _ in range(3):  # send a few times so the placement is not lost to UDP; the board stays fixed
    ue.sendUE4Pos(board_id, board_type, 0, TargePos, TargeAng)
    time.sleep(0.2)
print("checkerboard placed at (UE4 map frame):", TargePos)

# # Enter Offboard mode to start vehicle control
time.sleep(2)
for i in range(VehilceNum):
    MavList[i].initOffboard()

for i in range(VehilceNum):
    # take off to roughly the checkerboard altitude
    print("taking off...")
    MavList[i].SendVelNED(0, 0, -takeoff_speed)
    time.sleep(flight_height / takeoff_speed)

    # settle and turn to the base yaw so the front-left fisheye faces the static board
    for _ in range(int(2.0 / dt)):
        MavList[i].SendPosNED(0, 0, -flight_height, base_yaw)
        time.sleep(dt)

    # fly the excitation trajectory; the checkerboard stays fixed
    print("start flying the calibration trajectory (front-left fisheye)...")
    t = 0.0
    lastTime = time.time()
    while t < T_total:
        dx = A_xy * np.sin(w_xy * t)                       # figure-8 (lemniscate)
        dy = A_xy * np.sin(w_xy * t) * np.cos(w_xy * t)
        dz = A_z * np.sin(w_z * t)                         # altitude oscillation
        yaw_cmd = base_yaw + yaw_amp * np.sin(w_yaw * t)   # yaw oscillation around +45 deg

        target_x = dx                                      # trajectory centered on take-off point
        target_y = dy
        target_z = -flight_height - dz                     # z negative is up in NED

        MavList[i].SendPosNED(target_x, target_y, target_z, yaw_cmd)

        t += dt
        lastTime = lastTime + dt
        sleepTime = lastTime - time.time()
        if sleepTime > 0:
            time.sleep(sleepTime)
        else:
            lastTime = time.time()

    print("data collection finished.")
