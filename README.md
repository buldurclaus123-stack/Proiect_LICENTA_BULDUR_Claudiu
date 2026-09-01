# Person Identification, Active Positioning, and Distance Estimation Using Thermal Imaging and LiDAR

## Overview
This repository contains the graduation project developed for the Faculty of Industrial Engineering and Robotics (FIIR), National University of Science and Technology POLITEHNICA Bucharest. The project focuses on an autonomous night-patrol security system that integrates multi-sensor data fusion, Edge AI, and an active mechanical positioning mechanism to detect intruders and calculate their 3D spatial coordinates in real-time under zero-visibility conditions.

## Key Features
- **Thermal Imaging Acquisition:** Capturing real-time heat signatures using an InfiRay Mini 2 thermal camera (Germanium lens, VOx microbolometer) for robust operation in pitch darkness or degraded visibility.
- **Active Pan/Tilt Positioning:** Electro-mechanical scanning mechanism driven by a NEMA 17 stepper motor, a TB6600 driver, and an Arduino Mega, controlled via an absolute home position (microswitch).
- **LiDAR Telemetry & 3D Localization:** Real-time distance measurement using a YDLIDAR X4 2D LiDAR sensor combined with geometric 3D Pythagorean transformations and sensor offsets.
- **Edge AI Object Detection:** Custom-trained YOLOv11 pipeline optimized and deployed locally on an NVIDIA Jetson Orin Nano for instant target identification (over 26 FPS).
- **Live Dispatch Transmission:** Asynchronous real-time video and telemetry streaming to a security dispatch center using the MediaMTX server and the SRT (Secure Reliable Transport) protocol.

## Tech Stack & Tools
- **Languages:** Python, C/C++ (Arduino)
- **Computer Vision & ML:** OpenCV, Ultralytics YOLO (v11), PyTorch, Edge AI (NVIDIA Jetpack)
- **Point Cloud / Sensor Libraries:** 
  - OpenCV (`cv2`)
  - Ultralytics YOLO (`YOLO`)
  - NumPy (`numpy`)
  - PySerial (`serial`)
  - Struct (`struct`)
  - Threading / Event (`threading`)
- **Hardware & Microcontrollers:** NVIDIA Jetson Orin Nano, Arduino Mega 2560, TB6600 Stepper Driver, NEMA 17 Motor
- **CAD & Mechanical Integration:** SolidWorks (custom control and transmission casings with worm gear design), 3D Printing (PETG)

## System Architecture & Workflow
1. **Detection:** The thermal camera captures frames, and the YOLOv11 model detects intruders (bounding boxes).
2. **Targeting:** The system computes the angular deviation ($\phi$) from the camera center.
3. **Telemetry Fusion:** A dynamic angular window is queried from the LiDAR scan data, and a median filter isolates the true distance.
4. **3D Spatial Mapping:** Real-time 3D coordinates are calculated using spatial offsets ($\Delta X, \Delta Z$) and distance formulas.
5. **Action & Stream:** Data is logged and streamed live to the control room.
##  Setup & Hardware Acceleration (NVIDIA Jetson Orin Nano)

This project relies on hardware acceleration (CUDA 12.2 / TensorRT) provided by **JetPack 6.2**. Do **NOT** install PyTorch from standard PyPI, as it will overwrite the CUDA-enabled ARM64 binaries.
# Spatial Analysis and Center of Equilibrium Tracker

## Overview FOR "COD_Testare_YOLO"
This script demonstrates the analytical capabilities of the detection system. Instead of real-time camera processing, it runs inference on a pre-recorded video file to perform advanced spatial calculations. 

**Key functionalities:**
* **Center of Equilibrium:** Calculates the dynamic geometric center (centroid) between all detected objects in a frame.
* **Angular Deviation:** Computes the exact angle (in degrees) of each target relative to the camera's optical center.
* **Pixel-to-Meter Conversion:** Estimates spatial distances using a fixed conversion ratio.
* **Telemetry Export:** Automatically logs all frame-by-frame data (classes, X/Y coordinates, distances, and angles) into a structured `raport_final_complet.csv` file for post-mission analysis.


## Setup & Installation

Clone the repository and install the required dependencies:

```bash
git clone [https://github.com/buldurclaus123-stack/Proiect_LICENTA_BULDUR_Claudiu.git]
cd repo-licenta
pip install -r requirements.txt
python src/main.py