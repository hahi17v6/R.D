# 🤖 R.D - Autonomous Delivery Robot: Technical Reference Manual

[![Project Status: Active](https://img.shields.io/badge/Project%20Status-Active-brightgreen.svg)]()
[![Hardware: ESP32 & Arduino](https://img.shields.io/badge/Hardware-ESP32%20%26%20Arduino-blue.svg)]()
[![Tech: A* Navigation](https://img.shields.io/badge/Tech-A*%20Navigation-orange.svg)]()

This document provides a comprehensive technical overview of the **Autonomous Delivery Robot (R.D)** project. It covers the architecture, communication protocols, algorithms, hardware specifications, and software logic.

---

## 📑 Table of Contents
1.  [System Architecture](#-system-architecture)
2.  [Communication Protocols (UART Binary)](#-communication-protocols-uart-binary)
3.  [Navigation & Pathfinding (A*)](#-navigation--pathfinding-a)
4.  [Positioning & ArUco Recalibration](#-positioning--aruco-recalibration)
5.  [Hardware Specification & Pinout](#-hardware-specification--pinout)
6.  [Software State Machine](#-software-state-machine)
7.  [Web Interface & API Reference](#-web-interface--api-reference)
8.  [Safety & Fail-safes](#-safety--fail-safes)

---

## 🏗 System Architecture

The robot operates on a **Distributed Control System** (DCS) using two microcontrollers:

### 🧠 ESP32 Master (High-Level Control)
*   **Networking**: Hosts the Web Dashboard and WebSocket server.
*   **Pathfinding**: Runs the A* algorithm on a RAM-allocated grid map.
*   **Vision Interface**: Receives ArUco detections from an external mobile device via HTTP API.
*   **User Interface**: Manages the 4x4 Keypad and generates delivery security codes.
*   **Mission Orchestration**: Maintains the global state machine and handles the delivery queue.

### ⚙️ Arduino Uno Slave (Low-Level Control)
*   **Motion Control**: Manages DC motors with differential drive and speed ramping.
*   **Sensor Fusion**: Reads 4 ultrasonic sensors and an MPU6050 IMU.
*   **Local Safety**: Implements autonomous obstacle avoidance and wall centering.
*   **Peripheral Interface**: Controls the MG90S locking servo and I2C LCD status display.

---

## 📡 Communication Protocols (UART Binary)

Communication between the ESP32 and Arduino is handled via **Serial2 (115200 baud)** using structured binary packets to minimize overhead and latency.

### ⬆️ Command Packet (ESP32 → Arduino)
**Header**: `0xAA` | **Size**: 6 bytes
| Byte | Field | Description |
| :--- | :--- | :--- |
| 0 | Header | Constant `0xAA`. |
| 1 | Action | 1:FWD, 2:BACK, 3:LEFT, 4:RIGHT, 5:STOP, 6:UNLOCK, 7:OPEN_SERVO, 8:CLOSE_SERVO, 9:LCD_MSG. |
| 2-5| Argument | 32-bit integer (e.g., specific LCD message ID or target angle). |

### ⬇️ Sensor Packet (Arduino → ESP32)
**Header**: `0xBB` | **Size**: 12 bytes
| Byte | Field | Description |
| :--- | :--- | :--- |
| 0 | Header | Constant `0xBB`. |
| 1-4 | Distances | Left, Right, Front, Back (0-255 cm). |
| 5-6 | IMU Angle | 16-bit signed integer (degrees). |
| 7 | Button | Push-button state (0/1). |
| 8 | Event | 0:None, 1:Obstacle, 2:HatchOpened. |
| 9 | Delta Dist | Signed 8-bit integer (cm since last update) for odometry. |

---

## 🗺 Navigation & Pathfinding (A*)

The robot uses a modified **A* Algorithm** (`AStarPathfinder.h`) optimized for ESP32 memory constraints.

### 🟢 Grid Configuration
*   **Cell Size**: $20 \text{cm} \times 20 \text{cm}$ ($0.20$m).
*   **Map Source**: `map.bin` on SD card (bit-packed grid).
*   **RAM Cache**: Downsampled "coarse map" stored in RAM (max 50,000 cells / ~50KB).

### 🟡 Algorithm Details
*   **Connectivity**: 8-direction movement (Octile distance heuristic).
*   **Corner Safety**: Prevents diagonal movement if adjacent cardinal cells are occupied.
*   **Robustness**: If a destination is inside a wall, the algorithm automatically searches for the nearest free cell within a 5-cell radius.
*   **Path Memory**: Supports paths up to 300 steps (`MAX_PATH`).

---

## 🎯 Positioning & ArUco Recalibration

To combat wheel slip and odometry drift, the robot uses **ArUco Markers** as absolute global anchor points.

### 📍 Recalibration Logic
When a marker is detected (via the mobile phone camera API), the robot:
1.  Lookup marker ID in the `balises[]` registry.
2.  Overrides current $(X, Y)$ coordinates and orientation $(\theta)$ with the marker's calibrated world position.
3.  Notifies the Dashboard via WebSocket for real-time map visualization update.

### 🚩 Key Landmarks
*   **Marker 29**: Kitchen / Home Base.
*   **Markers 24-42**: Individual Patient Rooms.
*   **Markers 201-213**: Corridor intersections and technical waypoints.

---

## 🛠 Hardware Specification & Pinout

### 🔋 ESP32 Master
| Peripheral | Pin | Responsibility |
| :--- | :--- | :--- |
| **SD Card (SPI)** | CS:4, MOSI:23, SCK:18, MISO:19 | Map & Site hosting. |
| **UART Serial2** | TX:17, RX:16 | Inter-MCU link. |
| **Keypad Rows** | 32, 33, 15, 26 | Input. |
| **Keypad Cols** | 27, 14, 12, 13 | Input. |

### ⚙️ Arduino Slave
| Peripheral | Pin | Responsibility |
| :--- | :--- | :--- |
| **Motors (A/B)** | Dir:12/13, PWM:3/11, Fren:9/8 | Movement. |
| **Ultrasonic** | L:4, R:5, F:6, B:7 | Obstacle detection. |
| **Servo** | 10 | Hatch Lock. |
| **Button** | A0 | Delivery confirmation. |
| **I2C LCD/IMU** | SDA/SCL | Display & Gyro. |

---

## 🔄 Software State Machine

The robot operates on 7 core states (`MissionState`):

1.  **`MISSION_IDLE`**: Awaiting command from Dashboard.
2.  **`GOING_TO_KITCHEN`**: Moving to pick-up point (`Marker 29`).
3.  **`WAITING_LOADING`**: Stopped at base; waits for staff confirmation via WebSocket.
4.  **`GOING_TO_ROOM`**: Navigating to the patient's room marker.
5.  **`WAITING_DELIVERY`**: Stopped at room; checks for Keypad code entry.
6.  **`RETURNING_HOME`**: Mission success; returning to base.
7.  **`NEEDS_CHARGE`**: Battery $< 20\%$; status locked until charged.

---

## 🌐 Web Interface & API Reference

The ESP32 hosts a **Full-stack Single Page Application** (`index.html`) with dual-role authentication.

### 🔑 Authentication
*   **Staff**: `admin` / `robot2024` or `infirmier` / `soins123`.
*   **Patient**: Authenticated via Room Number + PIN (last 4 digits of room).

### 📡 API Endpoints
*   `GET /aruco?id=<id>`: Triggers ArUco marker detection logic.
*   `GET /status`: Returns JSON of current position, state, and sensors.
*   `POST /auth?role=<role>`: Authenticates user credentials.

---

## 🛡 Safety & Fail-safes

### 🛑 Obstacle Avoidance Sequence
Triggered when Front sensor $< 20 \text{cm}$:
1.  **Emergency Stop** + Notify Dashboard.
2.  **Wait 5s** for obstacle to move.
3.  **Active Evasion**: If blocked, detects freer side (Left vs Right), pivots $90^\circ$, advances, and re-routes.

### 🐕 Watchdogs
*   **ESP32 Watchdog (30s)**: Panic reset if mission logic hangs.
*   **Arduino Watchdog (3s)**: Performs emergency stop if UART heartbeat (`0xAA` packet) from ESP32 is lost.
*   **Mission Timeout (10m)**: Automatically cancels mission and returns home if target is not reached within 10 minutes.
