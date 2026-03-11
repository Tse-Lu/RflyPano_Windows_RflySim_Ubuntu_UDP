Communication between RflySim in Windows and Ubuntu in NVIDIA Jetson NX by UDP. Images and IMU data provided by RflySim are used by NX.

Click and watch the following example:
[![watch the example](./img/overview.png)](https://www.youtube.com/watch?v=y5ORxYG_ocE)

# Usage

### Windows
In `Windows/`.
1. Edit `config/Config_ROS.json` for the configuration of vision sensors. 
2. Run `SITLRun` or `SITLRunUE5` in RflySim to launch the simulation process.
3. Run `getRflysimData.py` to generate image and IMU data and transfer these data to NX by UDP.

### Ubuntu
In `Ubuntu/`
1. Create a ROS workspace and compile.
2. Run `roslaunch rflysim_ros_pkg four_fisheye.launch` to get and publish the images (fisheye (`/camera/LeftBack`,`/camera/LeftFront`, `/camera/RightBack`, `/camera/RightFront`) and pinhole (`/camera/Pinhole`)).
3. Run `roslaunch rflysim_ros_pkg imu.launch` to get and publish the IMU data (`/imu`).
4. Run `roslaunch rflysim_4fisheye_2_pano_pkg four_fisheye_2_pano.launch` to stitch and publish the panoramic images (`/panorama`).
5. The timestamps of images and IMU data are time relative to the time origin of RflySim platform. 

# Warning
1. The users need to create `CopterSim1.csv` in `/PX4PSP/CopterSim/` (directory of RflySim installation), where `1` represents the index of the target UAV. Then, the groundtruth will be recorded in this file after simulation.

# Acknowledge
If you find this repo useful, please cite:

```bibtex
@inproceedings{RflyPano,
  title={RflyPano: A Panoramic Benchmark for Ultra-low Altitude UAV Localization Powered by RflySim},
  author={Dai, Dun and Lu, Ze and Dai, Xunhua and Quan, Quan},
  booktitle={Proceedings of the AAAI Conference on Artificial Intelligence},
  year={2026}
}
```