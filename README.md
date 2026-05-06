# Multi-Channel BEV Perception Pipeline (Core Engine)

<p align="center">
  <!-- Assurez-vous que le chemin est exact. Si le repo est sur GitHub, c'est relatif à la racine -->
  <img src="results/kitti-bev.gif" width="600" alt="Bird's Eye View Pipeline"/>
</p>

## 📌 Project Overview
This repository implements a high-performance **Bird's Eye View (BEV)** perception engine. It transforms raw 3D LiDAR point clouds into multi-channel 2.5D feature maps, optimized for real-time obstacle detection on edge computing hardware (benchmarked on **AMD Athlon 300U / 8GB RAM**).

## 🚀 Core Algorithmic Pipeline

### 1. Multi-Modal Feature Extraction
The engine encodes raw LiDAR data into 4 synchronized feature maps:
*   **Height Map:** Isolate vertical structures from the ground plane.
*   **Intensity Map:** Capture surface reflectivity for rail and lane marking detection.
*   **Density Map:** Measure structural solidity to filter out environmental noise (smoke, dust, vegetation).
*   **RGB-LiDAR Fusion:** Reproject camera pixels onto 3D points using intrinsic/extrinsic calibration matrices.

### 2. Robust Ground Filtering
Implemented a **Ground Plane Estimation** algorithm to segment the drivable surface from potential obstacles. (z value filtering)
*   **Benefit:** Reduces point cloud processing density, speeding up downstream object detection by approximately 40%.
*   **Application:** Enables clean obstacle isolation for clustering and tracking algorithms.

### 3. Kinematic Synchronization
Integration of **OXTS/GPS/IMU** telemetry data to compute real-time ego-motion
*   **Velocity Overlay:** Dynamic speed calculation (km/h) synchronized per frame
*   **Motion Compensation:** Correcting LiDAR "motion blur" caused by vehicle movement during sensor rotation

---

## 📂 Repository Structure
```PlainText
├── src/
│   └── kitti-bev.py           # Core processing logic & BEV generation
├── data/
│   ├── oxts/                  # GPS/IMU telemetry files (.txt)
│   └── velodyne_points/       # Raw LiDAR scans (.bin)
├── results/
│   ├── kitti-bev.mp4          # BEV with ground
│   └── kitti-bev_filtered.mp4 # BEV with filtered ground (z> -1.5 & z < 1.0)
│   └── kitti-bev.gif          # 
│   └── kitti-bev_filtered.gif
└── README.md
```

## 💻 Getting Started

### 1. Prerequisites
* **Python 3.8+**
* **FFmpeg** (required for `.mp4` to `.gif` conversion)
* **Dataset:** Download the [KITTI Odometry/Tracking dataset](https://www.cvlibs.net/datasets/kitti/) and place the sequences in the `data/` folder.

### 2. Installation
```bash
git clone [https://github.com/yourusername/kitti-bev-perception.git](https://github.com/yourusername/kitti-bev-perception.git)
cd kitti-bev-perception
pip install numpy opencv-python open3d pyyaml
```

### 3. Running the Pipeline
To process the LiDAR frames and generate the multi-channel BEV visualization:
```bash
python src/kitti-bev.py --data_path data/ --output_results results/
