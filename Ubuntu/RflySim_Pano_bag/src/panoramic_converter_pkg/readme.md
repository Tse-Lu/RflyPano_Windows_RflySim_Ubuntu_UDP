# Panoramic Image Transformer

A ROS package to project and remap Equirectangular Panoramic (ERP) images into multi-fisheye or omnidirectional catadioptric camera representations.

## Features
* **ERP to Quad-Fisheye:** Re-projects a $360^\circ$ panoramic image into four independent fisheye images with optical axes oriented toward **left-front, left-back, right-back, and right-front**.
* **ERP to Catadioptric:** Projects panoramic data into omnidirectional catadioptric images with a central mirror axis.

---

## Launch Files & Usage

### 1. Quad-Fisheye Generation
```
roslaunch panoramic_transformer pano_to_4fisheye.launch
```
- Description: Converts the panoramic topic into four distributed fisheye streams.

### 2. Downward Catadioptric Generation
```
roslaunch panoramic_transformer pano_to_catadioptric_down.launch
```
- Description: Transforms the panoramic image into a catadioptric image with the central axis pointing downward.（$X_c^{cata}Y_c^{cata}Z_c^{cata}\leftarrow Z_c^{pano}X_c^{pano}Y_c^{pano}$）

### 3. Upward Catadioptric Generation
```
roslaunch panoramic_transformer pano_to_catadioptric_up.launch
```
- Description: Transforms the panoramic image into a catadioptric image with the central axis pointing upward.

### 4. Image Decompression Utility
```
roslaunch panoramic_transformer panorama_compressed_to_raw.launch
```
- Description: Preprocessing node to decompress the panoramic topic if it is published as a compressed image stream.