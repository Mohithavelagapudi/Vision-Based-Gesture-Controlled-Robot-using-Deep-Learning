# 🤖 Vision-Based Gesture-Controlled Robot using Deep Reinforcement Learning

An end-to-end robotic control system built from scratch — integrating computer vision, deep learning, and motor control for real-time gesture-driven navigation.

----
### 🧠 Overview

This project implements a gesture-controlled mobile robot using computer vision (OpenCV + MediaPipe) and deep neural gesture recognition models (TensorFlow/Keras).
The robot interprets human hand gestures in real time to perform navigation tasks like move forward, reverse, turn left/right, and stop, leveraging a Raspberry Pi for onboard computation and motor control via an H-Bridge driver.

🛠️ **Built entirely from scratch** — from mechanical assembly and hardware wiring to computer vision pipeline and control logic — as part of a Reinforcement Learning & Robotics course capstone.

----
### 🎯 Key Features

| Feature                           | Description                                                                                                 |
| --------------------------------- | ----------------------------------------------------------------------------------------------------------- |
| **Real-time Gesture Recognition** | Detects hand landmarks using MediaPipe and classifies gestures using a deep CNN trained on custom datasets. |
| **Robotic Motor Control**         | Commands dual DC motors using PWM signals through an H-Bridge interface on a Raspberry Pi.                  |
| **Vision-Based Command Mapping**  | Gesture → Action mapping enables autonomous motion based purely on visual inputs.                           |
| **End-to-End Integration**        | Combines computer vision, deep learning, embedded control, and robotic hardware.                            |
| **Built-from-Scratch Robot**      | Complete mechanical build, wiring, and code integration designed by the team.                               |

----
### 🧩 System Architecture

```
graph TD
A[Camera Input (OpenCV)] --> B[Hand Landmark Detection (MediaPipe)]
B --> C[Gesture Classification (CNN Model - TensorFlow)]
C --> D[Gesture-to-Action Mapping Layer]
D --> E[Motor Control Unit (PWM + GPIO)]
E --> F[Robot Movement (H-Bridge + DC Motors)]
```

----

### ⚙️ Technical Stack

| Domain                    | Technologies                                                                                  |
| ------------------------- | --------------------------------------------------------------------------------------------- |
| **Computer Vision**       | OpenCV, MediaPipe, Contour Analysis, Background Subtraction                                   |
| **Deep Learning**         | TensorFlow, Keras (CNN Gesture Classifier)                                                    |
| **Embedded Systems**      | Raspberry Pi 4, MDD10A H-Bridge Motor Driver                                                  |
| **Programming Languages** | Python                                                                                        |
| **Hardware Control**      | RPi.GPIO for PWM and direction control                                                        |
| **Dataset**               | Custom hand gesture dataset (`ok`, `stop`, `thumbs up`, `peace`, etc.) collected for training |

----

### 🚀 Project Pipeline

1. **Data Acquisition**

- Collected gesture videos under varying lighting/backgrounds.

- Labeled gestures for classification model training.

2. **Model Training**

- Built and trained a CNN using TensorFlow to classify gestures.

- Optimized using transfer learning for limited dataset size.

3. **Computer Vision Processing**

- Used MediaPipe Hands for landmark extraction.

- Integrated OpenCV for preprocessing and segmentation.

4. **Gesture → Action Mapping**

- Mapped recognized gestures to robot control commands:

- ✋ Stop → Halt motors

- 👍 Move Forward

- 👎 Move Backward

- 🤟 Turn Right

- ✌️ Turn Left

5. **Motor Control**

- Controlled motors via GPIO PWM signals through MDD10A H-Bridge.

- Implemented smooth speed control and safe shutdown routines.

----
### 📸 Real Robot Build

🧩 Fully built and tested — below are snapshots from our final build and live testing sessions.

- ✅ Physical robot assembled using custom chassis, motor mount, and onboard camera.

- ✅ Controlled entirely via visual input — no manual remote or wired control.
  
----

### 🧪 Experimental Results

| Gesture      | Model Accuracy | Latency (ms) | Action       |
| ------------ | -------------- | ------------ | ------------ |
| 👍 Thumbs Up | 98.2%          | 43           | Move Forward |
| ✋ Stop       | 99.1%          | 39           | Halt         |
| ✌️ Peace     | 97.3%          | 41           | Turn Left    |
| 🤟 Rock      | 96.7%          | 45           | Turn Right   |


Overall System FPS: ~20 FPS on Raspberry Pi 4
Inference Latency: <50ms (real-time gesture recognition)

----
### 🧰 Hardware Used

| Component                         | Purpose                                     |
| --------------------------------- | ------------------------------------------- |
| **Raspberry Pi 4 (4GB)**          | Primary compute unit for vision and control |
| **Pi Camera Module**              | Real-time image capture                     |
| **MDD10A Dual Motor Driver**      | PWM-based control of DC motors              |
| **DC Motors x2**                  | Left and right wheel drive                  |
| **Custom Chassis + Battery Pack** | Robot structure and power supply            |

---

### 🪄 Example Demonstration



