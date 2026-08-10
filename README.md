#Tamagotchi Triplets
WRO 2026 FUTURE ENGINEERS
This is the repository for GJ504b competing in the World Robot Olympiad, Future Engineer Category in 2026


THE TEAM 

| Name | Skills | Role |
|-----------------|-----------------|-----------------|
1. Abigail Maina | add your skills | add your role in the team | 
2. Tawana Chinoruma | add your skills | add your role in the team |
3. Michael Sambaza | add your skills | add your role in the team | 

General Information: 
All 3 of us are 18 years old and Upper Sixth Students at Saint John's College in South Africa
We are competing in the WRO for the first time
We are friends in most of the same A Level classes who are passionate about STEM and are taking on the Future Engineer Category to challenge ourselves and improve our skills

Team Name:
"Tamogatchi" refers to popular handheld digital pet beloved by Tawana."Triplets" was chosen because there are 3 of us


THE TASK

Information on rules and problem descriptions from the official WRO can be found here: 
https://wro-association.org/wp-content/uploads/WRO-2026-Future-Engineers-Self-Driving-Cars-General-Rules.pdf

We will be judged on 3 categories:
1. The Open Challenge: The vehicle must complete three (3) laps on the track with random placements
of the inside track walls.
2. Obstacle Challenge: The vehicle must complete three (3) laps on the track with randomly placed
green and red traffic signs. The traffic signs indicate the side of the lane the vehicle must follow.
The traffic sign to keep to the right side of the lane is a red pillar. The traffic sign to keep to the
left side of the lane is a green pillar. The vehicle should not move any of the traffic signs. After
the robot completed the three rounds, it had to find the parking lot and has to perform parallel
parking.
3. Documentation: In addition to designing and programming the vehicle,
teams must provide documentation that presents their engineering progress, the final vehicle
design, and final vehicle source code. This documentation must be uploaded to the public GitHub repository
  
> **Hardware**: Raspberry Pi 3B+ · Camera Module 3 · Arduino Uno  
> **Languages**: Python 3 · C++ (Arduino)
## 📚 Table of Contents

1. [System Overview](#-system-overview)
2. [Mobility & Mechanical Design](#-mobility--mechanical-design)
3. [Sensor & Vision Architecture](#-sensor--vision-architecture)
4. [Software Architecture](#-software-architecture)
5. [Key Engineering Decisions](#-key-engineering-decisions--trade-offs)
6. [Testing & Iteration History](#-testing--iteration-history)
7. [Build & Run Instructions](#-build--run-instructions)

## 🧠 System Overview

The **Tamagotchi-Triplets** autonomous vehicle is purpose‑built for the WRO 2026 Future Engineers Self‑Driving Car challenge. It uses a **dual‑processor architecture**:

| Processor | Role |
|-----------|------|
| **Raspberry Pi 3B+** | Computer vision (OpenCV), high‑level decision making, state machine |
| **Arduino Uno**      | Low‑level motor control, sensor filtering, serial command parsing |

The two boards communicate via USB Serial **(specify the baud, but I think it is 115200 baud)**. The Pi processes frames at 15–20 FPS and sends steering/speed commands to the Arduino, which translates them into PWM signals for the servo and DC motor.

Our vision pipeline is split into four logical modules, each feeding a central **Finite State Machine (FSM)** that implements the behaviour described in the official *Bot States* document:

- **Leaving Parking** – tightest turn from the magenta box
- **Straight Driving** – lane centring with adaptive offset
- **Turning Corners** – sharp 90° turns triggered by blue/orange lines
- **Traffic Obstacles** – pass red pillars on the right, green on the left
- **Final Parking** – align between the two magenta bars

The system is designed to handle **both track widths** (1000 mm and 600 mm) without manual recalibration, using a dynamic ROI and normalised lane offset.

## Mobility & Mechanical Design

### Chassis & Drive Train

[ ABBY - I SUMMON YOU]

### Weight & Dimensions

| Measurement | Value |
|-------------|-------|
| Length      | N/A |
| Width       | N/A |
| Height      | N/A|
| Weight      | N/A|

[commentary]

### Power Budget

We use a **dual‑battery system** to isolate noisy loads from sensitive electronics:

| Battery | Purpose | Specs |
|---------|---------|-------|
| **Battery 1** | N/A | N/A |
| **Battery 2** | N/A | N/A |

**Calculated Peak Current Draw**: (I'll double check these values)

| Component          | Typical Current | Peak Current |
|--------------------|-----------------|--------------|
| Raspberry Pi 3B+   | 700 mA          | 850 mA       |
| Pi Camera Module 3 | 150 mA          | 180 mA       |
| Arduino (what type?)     | 80 mA           | 150 mA       |
|  ESP32   |        |        |
| **Total**          | **number**      | **number**   |

[additional commentary once finalised]

---
## Sensor & Vision Architecture

### Sensor Suite

| Sensor             | Quantity | Purpose |
|--------------------|----------|---------|
| **Pi Camera 3W**   | 1        | Primary vision – lanes, signs, parking |
| **sensor type**        | 3     | Front + Left + Right ultrasonic (wall detection) |
| ****  |      |  |
| **** |     |  |

### Camera Configuration

- **Resolution**: `320 × 240` – balances detail and processing speed
- **Framerate**: `15 Hz` – stable and predictable
- **Exposure**: Manual (`shutter_speed = 10000 µs`, `ISO = 400`)
- **White Balance**: Manual (`AWB gains = 1.0, 1.0`) – locks colour consistency
- **Mount**: 

These settings prevent auto‑exposure flicker that would otherwise shift HSV thresholds mid‑run.

---

<br/>

## Software Architecture

### Finite State Machine (FSM)

Below is the **state machine**. The full detailed diagram with all conditions and transitions is available in [need to link the "Bot state" document .

~~~mermaid
graph TD
    A[START] --> B{Leaving Parking?}
    B -->|Magenta detected + Front USS < 5cm| C[Tightest Turn]
    C --> D[STRAIGHT]
    D --> E{Corner detected?}
    E -->|Blue / Orange line| F[Sharp Turn 90°]
    F --> D
    D --> G{Traffic Sign?}
    G -->|Red| H[Pass on Right]
    G -->|Green| I[Pass on Left]
    H --> D
    I --> D
    D --> J{3 Laps complete?}
    J -->|Yes| K[Parking]
    K --> L[STOP]
    J -->|No| D
~~~

---

## Vision pipeline modules
Our vision code (computervision.py) is structured into four logical modules, each corresponding to a state:

### Module A - Lane Keeping (detect_lane)
- Dynamic ROI (Region Of Interest): calculates the vanishing point where lane lines converge, cropping out irrelevant background
- Histogram peaks: find left and right lane boundaries
- Normalised Offset: -1.0 (left line) → 0.0 (centre) → +1.0 (right line). This ensures the PID controller works identically on 600 mm and 1000 mm tracks.
- Adaptive Slack: if lines are lost, S and V thresholds widen automatically, recovering from sudden lighting changes.
### Module B – Corner Detection (detect_corner)
Looks for thick blue (left turn) or orange (right turn) lines in the upper‑middle ROI.

Returns a boolean and the colour, triggering the "CORNER" state in the FSM.

### Module C – Traffic Obstacles (detect_traffic_sign)
Detects red or green pillars (50×50×100 mm).

Filters by aspect ratio (~1.0) and area (>150 pixels²).

Returns required_side – 'left' for green, 'right' for red – which the FSM uses to steer accordingly.

### Module D – Parking (detect_parking)
Looks for magenta bars (200×20 mm) with aspect ratio > 2.5.

Returns centroids of the two largest bars, used to align the car for parallel parking.



<br/>
