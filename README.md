#Tamagotchi Triplets
WRO 2026 FUTURE ENGINEERS
This is the repository for GJ504b competing in the World Robot Olympiad, Future Engineer Category in 2026


THE TEAM 

Names:
1. Abigail Maina
2. Tawana Chinoruma
3. Michael Sambaza

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

---

<br/>
