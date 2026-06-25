<!-- Communication between RflySim in Windows and Ubuntu in NVIDIA Jetson NX by UDP. Images and IMU data provided by RflySim are used by NX.

Click and watch the following example:
[![watch the example](./img/overview.png)](https://www.youtube.com/watch?v=y5ORxYG_ocE)

# Usage

### Windows
In `Windows/`.
1. Edit `config/Config_ROS.json` for the configuration of vision sensors. 
2. Run `SITLRun` or `SITLRunUE5` in RflySim to launch the simulation process.
3. Run `getRflysimData_xxx.py` to generate image and IMU data and transfer these data to NX by UDP. 
4. `getRflySimData_KalibrFisheyeFL.py` is used for calibrate fisheye camera (configured in `config/Config_Fisheye_FL.json`).

### Ubuntu
In `Ubuntu/`
1. Create a ROS workspace and compile.
2. Run `roslaunch rflysim_ros_pkg four_fisheye.launch` to get and publish the images (fisheye (`/camera/LeftBack`,`/camera/LeftFront`, `/camera/RightBack`, `/camera/RightFront`) and pinhole (`/camera/Pinhole`)).
3. Run `roslaunch rflysim_ros_pkg imu.launch` to get and publish the IMU data (`/imu`).
4. Run `roslaunch rflysim_4fisheye_2_pano_pkg four_fisheye_2_pano.launch` to stitch and publish the panoramic images (`/panorama`).
(2-4: `./RflySimTest.sh` )
5. The timestamps of images and IMU data are time relative to the time origin of RflySim platform. 

# Warning
1. The users need to create `CopterSim1.csv` in `/PX4PSP/CopterSim/` (directory of RflySim installation), where `1` represents the index of the target UAV. Then, the groundtruth will be recorded in this file after simulation.


# Acknowledge
If you find this repo useful, please cite:

```bibtex
@inproceedings{dai2026rflypano,
  title={RflyPano: A Panoramic Benchmark for Ultra-low Altitude UAV Localization Powered by RflySim},
  author={Dai, Dun and Lu, Ze and Dai, Xunhua and Quan, Quan},
  booktitle={Proceedings of the AAAI Conference on Artificial Intelligence},
  volume={40},
  number={22},
  pages={18216--18224},
  year={2026}
}
``` -->

This project enables seamless real-time communication between a Windows-based RflySim instance and an Ubuntu-based NVIDIA Jetson edge computing platform via UDP, providing synchronized image and IMU data for UAV panoramic localization research.

[![Watch the demo video](./img/overview.png)](https://www.youtube.com/watch?v=y5ORxYG_ocE)

---

# Quick Start

### 1. Windows Environment (RflySim)
Navigate to the `Windows/` directory:

1. **Configure Sensors**: Edit `config/Config_ROS.json` to configure vision sensor parameters as needed.
2. **Launch Simulation**: Start the simulation environment in RflySim using `SITLRun` or `SITLRunUE5`.
3. **Data Streaming**: Run `getRflysimData_xxx.py` to generate image and IMU data, streaming them to the Jetson NX via UDP.
4. **Calibration**: Use `getRflySimData_KalibrFisheyeFL.py` to perform fisheye camera calibration (configured via `config/Config_Fisheye_FL.json`).

### 2. Ubuntu Environment (NVIDIA Jetson)
Navigate to the `Ubuntu/` directory:

1. **Build the Workspace**: Create and compile your ROS workspace.
2. **Execution**: Run the provided script `./RflySimTest.sh` to initialize the following:
    * **Image Publication**: Publishes fisheye streams (`/camera/LeftBack`, `/camera/LeftFront`, `/camera/RightBack`, `/camera/RightFront`).
    * **IMU Data**: Publishes synchronized IMU data (`/imu`).
    * **Panoramic Stitching**: Executes `four_fisheye_2_pano.launch` to stitch and publish panoramic images (`/panorama`).

> **Note**: All image and IMU timestamps are synchronized relative to the RflySim platform time origin.

---

# Important Configuration
Before running the simulation, please ensure that a `CopterSim1.csv` file is created in `/PX4PSP/CopterSim/` (within your RflySim installation directory), where `1` denotes the target UAV index. This file will automatically log the ground truth pose data post-simulation.

---

# Citation
If you find this repository useful in your research, please cite our work:

```bibtex
@inproceedings{dai2026rflypano,
  title={RflyPano: A Panoramic Benchmark for Ultra-low Altitude UAV Localization Powered by RflySim},
  author={Dai, Dun and Lu, Ze and Dai, Xunhua and Quan, Quan},
  booktitle={Proceedings of the AAAI Conference on Artificial Intelligence},
  volume={40},
  number={22},
  pages={18216--18224},
  year={2026}
}
```