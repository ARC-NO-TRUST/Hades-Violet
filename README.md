# Hades - Violet

## CSSE4011 – Advanced Embedded Systems Project

**Team Members:**
- Hao Wu – 46129495
- Ryan Liu – 46936095
- Jiahong He – 47315389

---

## Overview

**Hades - Violet** is a smart traffic officer gesture recognition system designed to enhance autonomous vehicle operations in non-standard traffic scenarios. The system interprets hand gestures from traffic officers using a combination of sensor data and machine learning, facilitating real-time vehicle responses in environments lacking conventional traffic signals.

---

## System Architecture

The system comprises the following components:

- **Wearable Sensor Node (Nordic Thingy:52):** Captures accelerometer data from traffic officer gestures.
- **Processing Unit (Raspberry Pi 4B):** Fuses accelerometer data, mmWave radar with camera input to classify gestures using MediaPipe and OpenCV.
- **Display Unit (M5Core2):** Presents recognised gestures and proximity alerts to the vehicle operator.
- **Actuation Mechanism:** Controls vehicle movements via a pan-tilt servo system based on recognised gestures.
- **Audio Feedback:** Provides proximity indications through a speaker system.
- **Monitoring Dashboard (Grafana):** Visualises system outputs for real-time monitoring and analysis.

---

## Features

- **Real-Time Gesture Recognition:** Utilises MediaPipe and OpenCV for efficient and accurate gesture classification.
- **Sensor Fusion:** Combines data from the accelerometer, mmWave radar and camera to improve recognition reliability.
- **BLE Communication:** Employs Bluetooth Low Energy for seamless data transmission between system components.
- **User Interface:** Offers intuitive visual and auditory feedback to vehicle operators.
- **Data Visualisation:** Integrates with Grafana to provide comprehensive system monitoring.

---

## Repository Structure

```
csse4011Project/
├── base/             # Base station code for BLE data reception
├── dashboard/        # Grafana dashboard configuration
├── gesture_ai/       # Gesture recognition scripts using MediaPipe and OpenCV
├── include/          # Shared header files and definitions
├── lib/              # External libraries and dependencies
├── mobile/           # Mobile node code for BLE advertising
├── actuator/         # Actuator node - servo motor and speaker control scripts
├── viewer/           # M5Core2 UI and BLE listener code
├── .gitignore        # Git ignore file
├── LICENSE           # Project license (MIT)
└── README.md         # Project documentation
```

---

## Getting Started

### Prerequisites

- **Hardware:**
  - Nordic Thingy:52
  - Raspberry Pi 4B
  - M5Core2
  - USB Camera
  - Pan-Tilt Servo Mechanism
  - Speaker System
  - mmWave Radar

- **Software:**
  - Python 3.x
  - MediaPipe
  - OpenCV
  - Grafana
  - Zephyr RTOS (for embedded components)
  - BLE libraries compatible with Nordic devices

### Installation

1. **Clone the Repository:**
   ```bash
   git clone https://github.com/ARC-NO-TRUST/Hades-Violet.git
   cd Hades-Violet
   ```

2. **Set Up Virtual Environment:**
   ```bash
   python3 -m venv venv
   source venv/bin/activate
   ```

3. **Install Dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

4. **Configure Hardware Components:**
   - Flash the appropriate firmware to the Nordic Thingy:52 and M5Core2 devices.
   - Connect the USB camera and servo mechanism to the Raspberry Pi.
   - Ensure BLE communication is established between devices.

5. **Launch the System:**
   - Start the gesture recognition script on the Raspberry Pi.
   - Initialise the Grafana dashboard for monitoring.
   - Activate the M5Core2 display unit.

---

## Usage

1. **Wearable Sensor Activation:**
   - The traffic officer wears the Nordic Thingy:52, which captures accelerometer data corresponding to hand gestures.

2. **Gesture Recognition:**
   - The Raspberry Pi processes accelerometer, mmWave radar and camera data to classify gestures in real-time.

3. **Vehicle Response:**
   - Recognised gestures are transmitted to the vehicle's control system, prompting appropriate movements via the servo mechanism.

4. **Feedback Mechanisms:**
   - The M5Core2 displays the recognised gesture and proximity alerts.
   - The speaker system provides auditory cues for proximity warnings.

5. **Monitoring:**
   - The Grafana dashboard visualises system performance and logs for analysis.

---

## Contributing

Contributions are welcome! Please follow these steps:

1. **Fork the Repository**

2. **Create a Feature Branch:**
   ```bash
   git checkout -b feature/YourFeature
   ```

3. **Commit Your Changes:**
   ```bash
   git commit -m "Add your feature"
   ```

4. **Push to the Branch:**
   ```bash
   git push origin feature/YourFeature
   ```

5. **Open a Pull Request**

---

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

---

## Acknowledgements

- University of Queensland – CSSE4011 Advanced Embedded Systems
- Open-source libraries: MediaPipe, OpenCV, Zephyr RTOS
- Grafana for data visualization

---

For more information, please refer to the [project wiki](https://github.com/ARC-NO-TRUST/csse4011Project/wiki).
