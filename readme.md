This project enables seamless real-time communication between a Windows-based RflySim instance and an Ubuntu-based NVIDIA Jetson edge computing platform via UDP, providing synchronized image and IMU data for UAV panoramic localization research.

[![Watch the demo video](./img/overview.png)](https://www.youtube.com/watch?v=y5ORxYG_ocE)

---

# Quick Start

### 1. Windows Environment (RflySim)
Navigate to the `Windows/` directory:

1. **Configure Sensors**: Edit `config/Config_ROS.json` to configure vision sensor parameters as needed.
2. **Launch Simulation**: Start the simulation environment in RflySim using `SITLRun` or `SITLRunUE5`.
3. **Data Streaming**: Run `getRflysimData_xxx.py` to generate image and IMU data, streaming them to the Jetson NX via UDP.
4. **Calibration**: Use `getRflySimData_KalibrFisheyeFL.py` to perform fisheye camera calibration (configured via `config/Config_Fisheye_FL.json`). Run `rosrun kalibr kalibr_calibrate_cameras --bag RflySim_FisheyeLR_IMU_Kalibr.bag --topics /camera/LeftFront --models pinhole-equi --target checkboard.yaml`
```
checkboard.yaml
target_type: checkerboard
targetCols: 12
targetRows: 8
rowSpacingMeters:0.06
ColSpacingMeters: 0.06
```

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