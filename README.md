<div align="center">
  <img src="https://capsule-render.vercel.app/api?type=waving&color=0:0d1b2a,100:4361ee&height=220&section=header&text=SBR-X&fontSize=70&fontAlign=50&fontAlignY=38&animation=fadeIn&fontColor=ffffff&desc=Self%20Balancing%20|%20Line%20Following%20|%20Obstacle%20Avoidance%20|%20Real-Time%20PID&descAlign=50&descAlignY=62&descSize=19&descColor=e0e6ed&shadow=true" alt="Header" />
</div>

<h3 align="center">A Two-Wheeled Self-Balancing Robot with Autonomous Line Following and Obstacle Avoidance</h3>

<p align="center">
  <img src="https://img.shields.io/badge/Status-Active-4361ee?style=for-the-badge&labelColor=0d1b2a&logo=arduino&logoColor=f4f4f9" />
  <img src="https://img.shields.io/badge/Modes-3-4361ee?style=for-the-badge&labelColor=0d1b2a&logo=googleearth&logoColor=f4f4f9" />
  <img src="https://img.shields.io/badge/Control-PID-4361ee?style=for-the-badge&labelColor=0d1b2a&logo=codeforces&logoColor=f4f4f9" />
  <img src="https://img.shields.io/badge/Axes-6DOF-4361ee?style=for-the-badge&labelColor=0d1b2a&logo=nvidia&logoColor=f4f4f9" />
</p>

<p align="center">
  <img src="https://img.shields.io/badge/MCU-Arduino%20Uno-b8860b?style=for-the-badge&labelColor=0d1b2a&logo=arduino&logoColor=f4f4f9" />
  <img src="https://img.shields.io/badge/IMU-MPU6050-b8860b?style=for-the-badge&labelColor=0d1b2a&logo=chip&logoColor=f4f4f9" />
  <img src="https://img.shields.io/badge/Dependencies-Zero%20External-b8860b?style=for-the-badge&labelColor=0d1b2a&logo=cplusplus&logoColor=f4f4f9" />
  <img src="https://img.shields.io/badge/License-MIT-6a040f?style=for-the-badge&labelColor=0d1b2a&logo=opensourceinitiative&logoColor=f4f4f9" />
</p>

<img width="100%" src="https://capsule-render.vercel.app/api?type=rect&color=0:0d1b2a,100:4361ee&height=3">


<br>

## Overview

SBR-X is a two-wheeled robot that balances itself upright in real time while offering two selectable autonomous driving modes: **line following** and **obstacle avoidance**. An IMU feeds tilt data into a PID control loop running on the main controller, which drives a pair of DC motors through a motor driver to keep the chassis balanced at all times, whether idle, tracking a line, or steering around obstacles.

All balancing, sensing, and motion logic runs on a single microcontroller. No external computer, no cloud, no build step needed, flash it once and it runs standalone.

<br>

<table width="100%">
<tr>
<td width="25%" valign="top" align="center">

**Self-Balancing**
<br>
<sub>Closed-loop PID keeps the chassis upright on two wheels, correcting tilt continuously</sub>

</td>
<td width="25%" valign="top" align="center">

**Line Following**
<br>
<sub>Reads a reflectance array to track a line while staying balanced</sub>

</td>
<td width="25%" valign="top" align="center">

**Obstacle Avoidance**
<br>
<sub>Dual ultrasonic sensors detect and steer around obstacles ahead</sub>

</td>
<td width="25%" valign="top" align="center">

**One-Button Modes**
<br>
<sub>Cycles between all three modes without reflashing firmware</sub>

</td>
</tr>
</table>

<br>

## Hardware

<p align="center">
  <img width="50%" alt="SBR-X Chassis" src="assets/robot.png" />
</p>

<p align="center"><sub>SBR-X chassis: battery deck (top) · driver, IMU &amp; ultrasonic stage (middle) · dual-wheel drive base (bottom)</sub></p>

<br>

## System Architecture

```mermaid
%%{init: {'theme':'base', 'themeVariables': { 'primaryTextColor':'#f4f4f9', 'lineColor':'#4361ee', 'fontFamily':'"Segoe UI", Helvetica, Arial, sans-serif', 'fontSize':'15px'}}}%%
flowchart LR
    A1["MPU6050<br/>Tilt + Gyro"] --> B1["Complementary<br/>Filter"]
    A2["Ultrasonic L/R<br/>Distance"] --> B2["Mode Logic"]
    A3["IR Array<br/>Line Position"] --> B2
    B1 --> B3["PID Balance<br/>Loop"]
    B2 -->|"angle bias"| B3
    B3 --> C1["Motor Driver"]
    C1 --> C2["Left Motor"]
    C1 --> C3["Right Motor"]

    classDef sense fill:#1b263b,stroke:#0a0f18,stroke-width:2px,color:#f4f4f9
    classDef logic fill:#0d1b2a,stroke:#05090f,stroke-width:2px,color:#f4f4f9
    classDef core fill:#4361ee,stroke:#1d2f8f,stroke-width:3px,color:#ffffff
    classDef driver fill:#1b263b,stroke:#0a0f18,stroke-width:2px,color:#f4f4f9
    classDef motor fill:#b8860b,stroke:#7a5a05,stroke-width:2px,color:#0d1b2a
    class A1,A2,A3 sense
    class B1,B2 logic
    class B3 core
    class C1 driver
    class C2,C3 motor
```

Every actuation command traces back through the same balance loop, so line following and obstacle avoidance never override stability, they only steer it.

<br>

## Mode State Machine

```mermaid
%%{init: {'theme':'base', 'themeVariables': { 'primaryTextColor':'#f4f4f9', 'lineColor':'#4361ee', 'fontFamily':'"Segoe UI", Helvetica, Arial, sans-serif', 'fontSize':'15px'}}}%%
flowchart LR
    START(["Power On"]) --> BAL["Balance<br/><sub>default mode</sub>"]
    BAL -- "button press" --> LINE["Line Following"]
    LINE -- "button press" --> OBS["Obstacle Avoidance"]
    OBS -- "button press" --> BAL

    classDef startNode fill:#b8860b,stroke:#7a5a05,stroke-width:2px,color:#0d1b2a
    classDef bal fill:#4361ee,stroke:#1d2f8f,stroke-width:2px,color:#ffffff
    classDef line fill:#1b263b,stroke:#0a0f18,stroke-width:2px,color:#f4f4f9
    classDef obs fill:#6a040f,stroke:#3a0208,stroke-width:2px,color:#ffffff
    class START startNode
    class BAL bal
    class LINE line
    class OBS obs
```

Every mode balances continuously in the background. The button only changes which outer behavior biases the lean angle, balance is always active underneath.

<br>

## Control Loop

```mermaid
%%{init: {'theme':'base', 'themeVariables': { 'primaryTextColor':'#f4f4f9', 'lineColor':'#4361ee', 'fontFamily':'"Segoe UI", Helvetica, Arial, sans-serif', 'fontSize':'15px'}}}%%
flowchart TD
    A[Read IMU angle] --> B{Within safe<br/>tilt range?}
    B -- No --> C[Cut motor output<br/>fail-safe stop]
    B -- Yes --> D[Compute PID error<br/>Kp · Ki · Kd]
    D --> E[Apply mode bias<br/>line offset or obstacle steer]
    E --> F[Send PWM to<br/>motor driver]
    F --> A

    classDef sense fill:#1b263b,stroke:#0a0f18,stroke-width:2px,color:#f4f4f9
    classDef step fill:#4361ee,stroke:#1d2f8f,stroke-width:2px,color:#ffffff
    classDef decision fill:#b8860b,stroke:#7a5a05,stroke-width:2px,color:#0d1b2a
    classDef drive fill:#1b263b,stroke:#0a0f18,stroke-width:2px,color:#f4f4f9
    classDef stop fill:#6a040f,stroke:#3a0208,stroke-width:2px,color:#ffffff
    class A sense
    class D,E step
    class B decision
    class F drive
    class C stop
```

<br>

## Modes

| Mode | Description | Primary Sensor |
|---|---|---|
| **Balance** | Holds an upright position on two wheels, correcting for tilt in real time | MPU6050 IMU |
| **Line Following** | Tracks a dark line on a light surface (or vice versa) while balanced | IR reflectance array |
| **Obstacle Avoidance** | Detects obstacles ahead and steers around them while balanced | Dual ultrasonic sensors |

<br>

## Components

| Part | Role | Qty |
|---|---|---|
| Arduino Uno / Nano | Main controller, runs the PID loop and mode logic | 1 |
| MPU6050 (accelerometer + gyroscope) | Measures tilt angle and angular velocity for balancing | 1 |
| Dual ultrasonic sensor module | Forward obstacle detection | 1 |
| IR line sensor array | Line detection for line-following mode | 1 |
| L298N / TB6612FNG motor driver | Drives both DC motors from controller PWM signals | 1 |
| DC gear motors with wheels | Drive wheels | 2 |
| Li-ion battery pack | Onboard power for logic and motors | 1 |
| Acrylic chassis (tiered mounting plates) | Structural frame for all stages | 1 |
| Mode select push button | Switches between balance / line-follow / avoid | 1 |

<sub>Component list and wiring assumed from the build shown above. Update this table and the pin map below to match your final parts.</sub>

<br>

## Wiring

| Signal | Component | Controller Pin |
|---|---|---|
| SDA / SCL | MPU6050 | A4 / A5 |
| Trig / Echo (L) | Ultrasonic sensor, left | D2 / D3 |
| Trig / Echo (R) | Ultrasonic sensor, right | D4 / D5 |
| IR array (analog) | Line sensor | A0–A3 |
| IN1–IN4 | Motor driver | D6–D9 |
| ENA / ENB | Motor driver (PWM speed) | D10 / D11 |
| Mode button | Push button | D12 |

<sub>Pin assignments are placeholders. Replace with your actual wiring once finalized.</sub>

<br>

## Roadmap

```mermaid
%%{init: {'theme':'base', 'themeVariables': { 'cScale0':'#1b263b','cScale1':'#4361ee','cScale2':'#b8860b','cScale3':'#6a040f','cScale4':'#0d1b2a','cScaleLabel0':'#ffffff','cScaleLabel1':'#ffffff','cScaleLabel2':'#0d1b2a','cScaleLabel3':'#ffffff','cScaleLabel4':'#ffffff', 'fontFamily':'"Segoe UI", Helvetica, Arial, sans-serif'}}}%%
timeline
    title SBR-X Build Roadmap
    Phase 1 : Self-balancing core
            : PID tuning
    Phase 2 : Line-following mode
    Phase 3 : Obstacle-avoidance mode
    Phase 4 : Bluetooth / app control
    Phase 5 : Autonomous mapping
```

<br>

## Tech Stack

Arduino C++ firmware running on a single microcontroller. No external compute, no wireless dependency for core balancing, the robot is fully self-contained once flashed.

<br>

## Getting Started

1. Wire the components as listed in the [Wiring](#wiring) table.
2. Open the firmware in the Arduino IDE.
3. Install the MPU6050 and motor driver libraries used in the sketch.
4. Tune the PID constants (`Kp`, `Ki`, `Kd`) for your chassis weight and motor response.
5. Flash the board, place the robot upright, and power on.
6. Press the mode button to cycle between Balance, Line Following, and Obstacle Avoidance.

<br>

## Contributing

1. Fork the repo
2. Create a branch: `git checkout -b feature/your-feature`
3. Commit your changes
4. Open a pull request

<br>

## References

Ang, K. H., Chong, G., and Li, Y. *PID Control System Analysis, Design, and Technology.* IEEE Transactions on Control Systems Technology.
InvenSense. *MPU-6000 and MPU-6050 Product Specification.*

<br>

## License

Distributed under the MIT License. See `LICENSE` for details.

<br>

<div align="center">
<p>Built by <a href="https://github.com/fraisasghar">Frais Asghar</a></p>
</div>

<div align="center">
If this project was useful to you, consider giving it a star. ⭐

<p3 align="center"><sub>Built for the Robotics &amp; Embedded Systems community &nbsp;&middot;&nbsp; Happy building</sub></p3>
</div>

<img width="100%" src="https://capsule-render.vercel.app/api?type=rect&color=0:0d1b2a,100:4361ee&height=3">
