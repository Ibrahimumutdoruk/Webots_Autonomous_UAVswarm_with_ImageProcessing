# Webots Autonomous UAV Swarm Simulation

## Objective
The primary objective of this project is to develop and simulate a decentralized, autonomous Unmanned Aerial Vehicle (UAV) swarm system within the Webots environment. The system demonstrates advanced dynamic formation control, vision-based environmental interaction, and robust inter-drone communication. The algorithm is designed to manage swarm state transitions, autonomous navigation, and dynamic task allocation without central external commands.

<img width="1632" height="852" alt="Image1" src="https://github.com/user-attachments/assets/50704997-31cf-456b-952c-c4d227c17a19" />



## Core Capabilities & Showcase
To demonstrate our full simulation capacities and algorithmic framework, the following advanced features have been successfully implemented:

* **OpenCV-Based Image Processing:** Real-time visual data processing utilizing OpenCV for environmental awareness.
* **Autonomous QR Decoding:** On-the-fly QR code scanning and decoding to dynamically update mission parameters and routes.
* **Color Detection (HSV):** Advanced HSV color space filtering to detect designated Red and Blue landing zones for individual drone separation tasks.
* **Complex Formations:** The swarm can seamlessly and autonomously transition between strictly geometric formations:
  * **Arrowhead **
  * **Line **
  * **V-Shape **
* **Synchronized Maneuvers:** The swarm flawlessly executes collective **Pitch**, **Roll**, and **Yaw** rotational maneuvers while maintaining strict formation integrity around the center of mass.
* **Custom PID Stabilization:** An independent PID controller is implemented for each axis to maintain altitude, velocity, and attitude stability during aggressive maneuvers.

## Webots Environment and Sensor Configurations
To optimize simulation performance and replicate realistic operational constraints, the following specific configurations were applied within the Webots nodes:

* **Communication:** Swarm telemetry, state synchronization, and QR data sharing are strictly handled through Webots `Emitter` and `Receiver` nodes. This simulates real-world RF datalink constraints.
* **Camera Sensor Constraints:** To reduce computational load and simulate an asymmetric sensor payload, the downward-facing camera is enabled exclusively on the 2nd Drone (ID=1). 
* **Camera Specifications:** * Field of View (FOV): 70 degrees (1.22 radians).
  * Resolution: 1920x1080 (1080p).
* **Positioning:** Ground truth positioning is gathered via Webots `GPS`, `InertialUnit`, and `Gyro` nodes.

## Requirements
This project requires Python libraries such as OpenCV and NumPy for real-time image processing, QR decoding, and HSV color detection. These libraries enable the autonomous UAV swarm to perform PID-stabilized Pitch/Roll maneuvers and execute dynamic geometric formations like Arrowhead, Line, and V-Shape successfully.d. 

