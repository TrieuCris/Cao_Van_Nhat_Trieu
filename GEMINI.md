# Delta Robot Classification System

## Project Overview

This project is a comprehensive robotic sorting system that uses a **Delta Robot** to pick and place objects (fruits/cakes) from a moving conveyor belt based on visual classification. The system is divided into two main components: a High-Level PC Controller (Vision & Logic) and a Low-Level Robot Controller (Firmware).

### System Architecture

1.  **PC Application (`GUI_PC`)**:
    *   **Role:** The "Brain" of the system. Handles User Interface, Computer Vision, Motion Planning, and Orchestration.
    *   **Tech Stack:** Python 3.11+, PyQt6 (GUI), OpenCV (Vision), YOLOv8 (AI Detection), Numba (Kinematics), PySerial (Communication).
    *   **Key Logic:**
        *   **Vision:** Uses `YOLOv8-OBB` (Oriented Bounding Box) to detect object class, position, and orientation.
        *   **Tracking:** Implements a custom object tracker to follow items on the conveyor.
        *   **Scheduler:** A Time-Based Scheduler (`auto_mode_controller.py`) that predicts when objects will reach the picking area and coordinates the robot's movement to intercept them accurately.
        *   **Kinematics:** Inverse Kinematics (IK) calculations are accelerated using Numba for real-time performance.

2.  **Firmware (`STM32_ROBOTDELTA`)**:
    *   **Role:** The "Muscle" of the system. Executes precise motor movements and handles sensor inputs.
    *   **Target:** STM32F103 Microcontroller.
    *   **Tech Stack:** C, STM32 HAL, FreeRTOS (likely, or bare metal loop), CMake/STM32CubeIDE.
    *   **Function:** Receives G-code-like commands or block commands from the PC, drives stepper motors via timers, and manages the pump/gripper.

## Directory Structure

### `GUI_PC/` (Python Control Software)

*   **`main.py`**: Entry point of the application. Initializes the PyQt GUI and starts the main loop.
*   **`vision_system.py`**: Handles camera interaction, image processing, YOLO inference, and object tracking. Contains `CameraConfig`, `DeepLearningProcessor`, and `VideoThread`.
*   **`auto_mode_controller.py`**: The core automation logic. It schedules picking tasks based on object detection timestamps and conveyor speed.
*   **`robot_controller.py`**: Manages the state of the robot, sends commands to the STM32, and handles connection logic.
*   **`kinematics.py`**: (Inferred) Contains the mathematical models for the Delta Robot (Forward/Inverse Kinematics), optimized with Numba.
*   **`connection_manager.py`**: Handles serial communication with the STM32 (COM port management).
*   **`camera_calibration.json`**: Critical configuration file for camera intrinsics, distortion coefficients, and coordinate mapping (Pixel to mm).
*   **`constants.py`**: System-wide constants (dimensions, limits, UI settings).
*   **`best_obb_traicay_0656.pt`**: The trained YOLOv8 model for object detection.

### `STM32_ROBOTDELTA/` (Firmware)

*   **`Core/Src/`**: Source code (main.c, interruptions, logic).
*   **`Core/Inc/`**: Header files.
*   **`STM32_ROBOTDELTA.ioc`**: STM32CubeMX configuration file.
*   **`CMakeLists.txt`**: Build configuration for CMake.

## Getting Started

### Prerequisites

*   **Python:** 3.11 or higher.
*   **Drivers:** CH340/CP210x drivers for the STM32 serial connection.
*   **Camera:** USB Webcam (MSMF backend preferred on Windows).

### Installation (PC)

1.  Navigate to the `GUI_PC` directory.
2.  Install dependencies (create a virtual environment recommended):
    ```bash
    pip install PyQt6 opencv-python ultralytics numba pyserial numpy pillow
    # Note: Install PyTorch with CUDA support if you have an NVIDIA GPU for faster inference.
    ```

### Running the Application

1.  Connect the STM32 Robot Controller via USB.
2.  Connect the Webcam.
3.  Run the main script:
    ```bash
    cd GUI_PC
    python main.py
    ```

### Usage Guidelines

*   **Manual Mode:** Use the toggle in the GUI to enable Manual Mode. This allows jogging the robot, testing the pump, and controlling the conveyor manually.
*   **Auto Mode:**
    1.  Ensure the Robot is **Homed** (Click Home button).
    2.  Ensure the Camera is Calibrated (loaded from JSON).
    3.  Place the sorting tray.
    4.  Press **Run** (or the Hardware Start Button).
    5.  The system will automatically detect objects and pick-and-place them into the designated bins.

## Development Notes

*   **Coordinate Systems:**
    *   **Robot Frame:** Cartesian (X, Y, Z) in mm. Z=0 is usually the base plane or a specific reference height.
    *   **Vision Frame:** Pixels (u, v). Converted to Robot Frame via Homography matrix in `CameraConfig`.
*   **Performance:**
    *   Vision processing runs in a separate thread (`VideoThread`) to avoid freezing the GUI.
    *   `AutoModeController` uses `time.perf_counter()` for high-precision timing to synchronize with the moving conveyor.
*   **Safety:**
    *   Always ensure the Emergency Stop (ESTOP) is functional.
    *   The software has checks for "Robot Not Homed" and "Camera Not Ready".
