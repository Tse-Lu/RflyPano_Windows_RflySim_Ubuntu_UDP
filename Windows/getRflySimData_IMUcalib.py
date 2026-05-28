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


################### still #########################