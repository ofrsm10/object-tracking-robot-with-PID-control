# Object Tracking Robot with PID Control

An autonomous navigation robot that follows a path by detecting and interpreting yellow directional signs using computer vision and PID control algorithms.

## Overview

This project implements an autonomous robot capable of navigating through an unknown environment by reading visual cues. The robot uses computer vision to detect yellow signs and arrows, then employs a PID controller for precise object tracking and navigation.

## Features

- **Computer Vision**: Real-time object detection using OpenCV
- **PID Control**: Precise motor control for smooth tracking and navigation
- **Sign Recognition**: Detects directional arrows (left/right) and stop signs
- **Distance Calculation**: Uses focal length calculations for depth perception
- **Autonomous Navigation**: Follows a path without human intervention
- **Real-time Processing**: Live camera feed processing at 30 FPS

## Hardware Requirements

- **AlphaBot2 Robot Kit**
- **Raspberry Pi 3B+** (or newer)
- **Pi Camera Module**
- **MicroSD Card** (16GB or larger recommended)
- **Yellow markers/signs** for path marking

### Robot Specifications
- Wheel radius: 2.1 cm
- Wheelbase: 10.5 cm
- Default speed: 18.7 cm/s
- PWM range: 10-25 duty cycle

## Software Dependencies

### Python Libraries
```bash
pip install opencv-python
pip install numpy
pip install RPi.GPIO
pip install picamera
pip install pytesseract
```

### System Requirements
- Python 3.7+
- Raspberry Pi OS (Raspbian)
- Camera interface enabled

## Installation & Setup

1. **Clone the repository:**
   ```bash
   git clone https://github.com/ofrsm10/object-tracking-robot-with-PID-control.git
   cd object-tracking-robot-with-PID-control
   ```

2. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

3. **Enable camera interface:**
   ```bash
   sudo raspi-config
   # Navigate to Interface Options > Camera > Enable
   ```

4. **Hardware connections:**
   - Connect AlphaBot2 motors to designated GPIO pins
   - Mount Pi Camera securely
   - Ensure proper power supply

## Usage

1. **Prepare the environment:**
   - Place yellow directional signs along the desired path
   - Signs should be painted in bright yellow (HSV: H=25-48, S=95-255, V=130-255)
   - Use arrow shapes for left/right directions
   - Use circular shapes for stop commands

2. **Position the robot:**
   - Place the robot in front of the first sign
   - Ensure clear line of sight to the sign

3. **Run the program:**
   ```bash
   python3 Autonomous.py
   ```

4. **Stop the program:**
   - Press 'q' in the OpenCV window, or
   - Robot will automatically stop when it reaches a circular stop sign

## How It Works

### Detection Process
1. **Camera Capture**: Continuous video feed at 640x480 resolution
2. **Color Filtering**: HSV color space filtering to isolate yellow objects
3. **Edge Detection**: Canny edge detection and contour analysis
4. **Shape Recognition**: Identifies rectangular signs and arrow/circle shapes
5. **Distance Calculation**: Uses focal length formula: `distance = (known_width × focal_factor) / pixel_width`

### Navigation Algorithm
1. **Sign Detection**: Robot starts by detecting the first yellow sign
2. **Distance & Position Calculation**: Determines sign's distance and horizontal position
3. **PID Control**: Computes wheel speeds based on tracking error
4. **Approach**: Robot moves toward the sign using PID-controlled motors
5. **Direction Reading**: At ~50cm distance, reads arrow direction or stop command
6. **Turn Execution**: Performs appropriate turn maneuver
7. **Repeat**: Searches for and tracks the next sign

### PID Controller Parameters
- **Kp (Proportional)**: 2.2
- **Ki (Integral)**: 0.0002  
- **Kd (Derivative)**: 0.1
- **Control Loop**: 100Hz (dt = 0.01s)

## Technical Implementation

### Key Classes
- **`PID`**: Implements PID control algorithm for angular velocity
- **`MOTORS`**: Controls robot wheel motors via GPIO PWM
- **Main Functions**:
  - `findSign()`: Detects and locates yellow signs
  - `findDirection()`: Determines arrow direction or stop command
  - `Procces()`: Main control loop for navigation

### Computer Vision Pipeline
1. HSV color space conversion
2. Color range thresholding
3. Gaussian blur and noise reduction
4. Canny edge detection
5. Morphological operations (dilation/erosion)
6. Contour detection and analysis
7. Shape approximation and classification

## Configuration

### Adjustable Parameters

**Vision Parameters:**
```python
# HSV Color Range for Yellow Detection
h_min, h_max = 25, 48
s_min, s_max = 95, 255  
v_min, v_max = 130, 255

# Camera Calibration
focalFactor = 1650.83
knownWidth = 12  # cm (actual sign width)
```

**Motor Control:**
```python
MIN_PWM = 10    # Minimum motor speed
MAX_PWM = 25    # Maximum motor speed
V0 = 18.7       # Base forward velocity (cm/s)
```

**PID Tuning:**
```python
KP = 2.2        # Proportional gain
KI = 0.0002     # Integral gain  
KD = 0.1        # Derivative gain
```

## Troubleshooting

**Common Issues:**

1. **Robot not detecting signs:**
   - Check lighting conditions
   - Verify yellow color calibration
   - Ensure camera is properly focused

2. **Erratic movement:**
   - Adjust PID parameters
   - Check motor connections
   - Verify power supply stability

3. **Camera errors:**
   - Enable camera interface in raspi-config
   - Check camera cable connections
   - Verify camera module compatibility

4. **Import errors:**
   - Install missing Python dependencies
   - Check Python version compatibility
   - Verify GPIO library installation

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly on hardware
5. Submit a pull request

## License

This project is provided as-is for educational and research purposes.

## Acknowledgments

- Built using AlphaBot2 robot platform
- OpenCV computer vision library
- Raspberry Pi Foundation


