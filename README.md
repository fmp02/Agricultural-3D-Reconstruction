# Agricultural 3D Reconstruction using ROS2 and Open3D

## Overview

This project presents a complete pipeline for the 3D reconstruction of an agricultural environment using a mobile robot equipped with an RGB-D camera.

The objective is to generate a digital representation of an agricultural field by combining robot navigation, RGB-D sensing and 3D point cloud reconstruction techniques.

The proposed approach consists of two main stages:

- Data acquisition;
- 3D reconstruction.

The project was developed as part of a university robotics course.

---

# Pipeline

The complete pipeline is composed of two main phases.

## Data Acquisition

A mobile robot equipped with an RGB-D camera autonomously navigates through the agricultural environment while collecting:

- RGB images;
- Depth maps;
- Robot poses;
- Camera intrinsic parameters.

The acquired data are stored into a structured dataset.

## 3D Reconstruction

The reconstruction algorithm:

- loads the acquired dataset;
- generates a point cloud for each frame;
- transforms every point cloud into the global reference frame;
- fuses all point clouds;
- removes noise;
- generates the final 3D reconstruction.

```
RGB-D Camera
      |
      v
Data Acquisition
      |
      v
Dataset Generation
      |
      v
Point Cloud Creation
      |
      v
Point Cloud Fusion
      |
      v
Final 3D Reconstruction
```

---

# Repository Structure

```
Agricultural-3D-Reconstruction/

│
├── README.md
├── requirements.txt
├── LICENSE
│
├── acquisition/
│   └── data_collector.py
│
├── reconstruction/
│   └── reconstruction3D.py
│
├── example_output/
│   └── fusion_final.ply
│
├── images/
│
└── docs/
```

---

# Requirements

## Python

Python 3.x

## Required Python packages

- numpy
- opencv-python
- open3d

Install the required packages using:

```bash
pip install -r requirements.txt
```

## Required ROS2 packages

- rclpy
- sensor_msgs
- geometry_msgs
- nav_msgs
- cv_bridge

---

# Data Acquisition

The data acquisition stage is implemented by the ROS2 node:

```
data_collector.py
```

The node performs two main tasks.

## Robot Navigation

The robot autonomously moves towards a predefined target.

The control algorithm:

- computes the target direction;
- aligns the robot;
- moves forward;
- stops when the destination is reached.

## Data Collection

During navigation, the node acquires:

- RGB images;
- depth maps;
- robot poses.

One frame is stored every three control iterations.

Camera intrinsic parameters are also saved for the reconstruction stage.

---

# Dataset Structure

The acquisition process generates the following dataset:

```
reconstruction_data/

├── color/
├── depth/
├── poses/
└── camera_intrinsics.json
```

## color

RGB images acquired by the RGB-D camera.

## depth

Depth maps.

## poses

Robot poses stored as 4×4 transformation matrices.

## camera_intrinsics.json

Camera intrinsic parameters:

- image width;
- image height;
- focal lengths;
- principal point coordinates.

---

# 3D Reconstruction

The reconstruction stage is implemented in:

```
reconstruction3D.py
```

For each acquired frame, the algorithm:

1. Loads RGB image, depth map and robot pose;
2. Creates an RGB-D image;
3. Generates a local point cloud;
4. Transforms the point cloud from the camera frame to the robot frame;
5. Transforms the point cloud into the global reference frame;
6. Adds the point cloud to the global reconstruction.

After processing all frames:

- voxel downsampling is applied;
- statistical outlier removal is performed;
- the final point cloud is generated.

---

# Running the Project

## Step 1

Launch the simulation environment.

The mobile robot equipped with an RGB-D camera navigates through the agricultural scene.

## Step 2

Run the data acquisition node.

```bash
python3 acquisition/data_collector.py
```

The node automatically generates the dataset.

## Step 3

Run the reconstruction algorithm.

```bash
python3 reconstruction/reconstruction3D.py
```

## Step 4

The final point cloud is generated as:

```
fusion_final.ply
```

and displayed using Open3D.

---

# Output

The final output of the project is a colored 3D point cloud representing the agricultural environment.

The generated point cloud can be:

- visualized with Open3D;
- exported for further processing;
- used for agricultural analysis.

---

# Applications

Potential applications include:

- Precision agriculture;
- Digital twins;
- Crop monitoring;
- Biomass estimation;
- Environmental mapping;
- Autonomous navigation.

---

# Future Improvements

Possible future developments include:

- Multi-row reconstruction;
- ICP refinement;
- Loop closure techniques;
- TSDF volumetric fusion;
- Semantic segmentation;
- Real-world deployment.

---

# Authors

**Fabiana Placchi**

**Giada Butticè**

University Project

Academic Year 2025/2026

---

# Acknowledgements

This project was developed as part of a university robotics course and combines concepts from robotics, computer vision and 3D reconstruction.
