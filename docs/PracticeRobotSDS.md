# 3310 - Blackhawk Robotics

## 2026 Robot Software Design Sheet - Practice Robot

| | |
|---|---|
| **Version:** | 1.0 |
| **Date:** | 2/21/2026 |
| **Author(s):** | Paul D. Copioli |
| **Google Doc Link:** | Practice Robot SDS |

---

## Revision History

| Date | Author | Version | Summary of Change |
|------|--------|---------|-------------------|
| 2/21/2026 | PDC | 1.0 | Initial Release |

> *Blackhawks Robotics CONFIDENTIAL*

---

## 1. Robot Drive Base Dimensions

### Swerve Drive Mechanism Details

- **Module:** WCP Swerve X2c
- **Drive Ratio:** X3 - 11T Motor Pinion
- **Drive Ratio:** 5.89:1 (54 / 11 × 16 / 40 × 45 / 15)
- **Steering Ratio:** Standard

---

## 2. CAN Bus Device Information

The competition robot will use two CANivores for the 2026 game. The tables below give all the pertinent information for each subsystem CAN device. Details for each mechanism will be in section 4 below, as required.

> **Note:** For all CAN IDs for the second CANivore we have put unique IDs because the practice robot only has one CANivore.

### 2.1. RoboRIO CAN Bus Device Information

| CAN ID | Device Description | Motor / Device | Motor Rev / Output Rev | Motor Top Speed | Output Top Speed | Output Direction / Motor + | Axis Limits | Stator Current Limit |
|--------|-------------------|----------------|----------------------|-----------------|-----------------|---------------------------|-------------|---------------------|
| 0 | Climber | X60 | TBD | 95 RPS | TBD | TBD | TBD | TBD |

### 2.2. CANivore #1 Device Information - Drive Base & Left CANivore

| CAN ID | Device Description | Motor / Device | Motor Rev / Output Rev | Motor Top Speed | Output Top Speed | Output Direction / Motor + | Axis Limits | Stator Current Limit |
|--------|-------------------|----------------|----------------------|-----------------|-----------------|---------------------------|-------------|---------------------|
| 0 | Front Left Steering | X44 | 12.1 (88/16 × 22/10) | 125 RPS | 10.33 RPS | NA | NA | 80A |
| 1 | Front Left Drive | X60 | 5.8909:1 (54/11 × 16/40 × 45/15) | 95 RPS | 16.13 RPS | NA | NA | 80A |
| 2 | Front Left CANcoder | CANcoder | 12.1 (88/16 × 22/10) | NA | 10.33 RPS | NA | NA | NA |
| 3 | Front Right Steering | X44 | 12.1 (88/16 × 22/10) | 125 RPS | 10.33 RPS | NA | NA | 80A |
| 4 | Front Right Drive | X60 | 5.8909:1 (54/11 × 16/40 × 45/15) | 95 RPS | 16.13 RPS | NA | NA | 80A |
| 5 | Front Right CANcoder | CANcoder | 12.1 (88/16 × 22/10) | NA | 10.33 RPS | NA | NA | NA |
| 6 | Back Right Steering | X44 | 12.1 (88/16 × 22/10) | 125 RPS | 10.33 RPS | NA | NA | 80A |
| 7 | Back Right Drive | X60 | 5.8909:1 (54/11 × 16/40 × 45/15) | 95 RPS | 16.13 RPS | NA | NA | 80A |
| 8 | Back Right CANcoder | CANcoder | 12.1 (88/16 × 22/10) | NA | 10.33 RPS | NA | NA | NA |
| 9 | Back Left Steering | X44 | 12.1 (88/16 × 22/10) | 125 RPS | 10.33 RPS | NA | NA | 80A |
| 10 | Back Left Drive | X60 | 5.8909:1 (54/11 × 16/40 × 45/15) | 95 RPS | 16.13 RPS | NA | NA | 80A |
| 11 | Back Left CANcoder | CANcoder | 12.1 (88/16 × 22/10) | NA | 10.33 RPS | NA | NA | NA |
| 12 | Intake Deploy | X44 | 20.35714:1 (32/12 × 38/16 × 40/16 × 18/14) | 125 RPS | 22.92 RPS | Retract | 0 → 0.403 Rev (0 → 145 deg) | 40A |
| 13 | Intake Roller Motor 1 | X44 | 3.55556:1 (32/10 × 20/18) | 125 RPS | 35.15 RPS | Out-take | NA | 80A |
| 14 | Intake Roller Motor 2 | X44 | 3.55556:1 (32/10 × 20/18) | 125 RPS | 35.15 RPS | Out-take | NA | 80A |

### 2.3. CANivore #2 Device Information - Shooting CANivore

| CAN ID | Device Description | Motor / Device | Motor Rev / Output Rev | Motor Top Speed | Output Top Speed | Output Direction / Motor + | Axis Limits | Stator Current Limit |
|--------|-------------------|----------------|----------------------|-----------------|-----------------|---------------------------|-------------|---------------------|
| 20 | Left Floor Roller | X44 | 1.66667:1 (20/12) | 125 RPS | 75 RPS | Intake | NA | 80A |
| 21 | Left Vertical Feed Roller | X44 | 1.5:1 (18/12) | 125 RPS | 83.33 RPS | Intake | NA | 80A |
| 22 | Left Turret | X44 | 45.714:1 (32/11 × 220/14) | 125 RPS | 2.734 RPS | Positive (Left) | -220 deg → +220 deg | 10A (initial) |
| 23 | Left Hood | X44 | 71.8667:1 (294/18 × 44/10) | 125 RPS | 1.739 RPS | Positive (Up) | 10 deg → 35 deg | 80A |
| 24 | Left Shooter | X60 | 1:1 | 95 RPS | 95 RPS | Suck In | NA | 150A |
| 25 | Right Floor Roller | X44 | 1.66667:1 (20/12) | 125 RPS | 75 RPS | Outtake | NA | 80A |
| 26 | Right Vertical Feed Roller | X44 | 1.5:1 (18/12) | 125 RPS | 83.33 RPS | Outtake | NA | 80A |
| 27 | Right Turret | X44 | 45.714:1 (32/11 × 220/14) | 125 RPS | 2.734 RPS | Positive (Left) | -220 deg → +220 deg | 80A |
| 28 | Right Hood | X44 | 71.8667:1 (294/18 × 44/10) | 125 RPS | 1.739 RPS | Positive (Up) | 10 deg → 35 deg | NA |
| 29 | Right Shooter | X60 | 1:1 | 95 RPS | 95 RPS | Suck In | NA | 150A |

---

## 3. Subsystem Detailed Information

### 3.1. Swerve Steering Axis

No additional information.

### 3.2. Swerve Drive Axis

- **Axis Top Speed:** 16.13 RPS
- **Axis Wheel Diameter:** 4 inches
- **Axis Linear Distance to Motor Ratio:** 2.133 inches per motor rev
- **Theoretical Top Robot Speed:** 16.88 ft/sec

### 3.3. Intake Rollers

- **# of Motors:** 2, independently controlled. Do not use follower mode
- **Control Mode:** Velocity TorqueCurrentFOC
- **Axis Gear Ratio:** 3.55556:1 (32/10 × 20/18)
- **Axis Rev per Motor Rev:** -0.28125 (10/32 × 18/20)
- **Axis Top Speed:** 35 RPS (2,100 RPM)
- **Axis Drum Diameter:** 1.325 in
- **Axis Linear Travel to Motor Ratio:** 1.1707 in per motor rev (0.28125 × π × 1.325)
- **Axis Linear Top Speed:** 146 in/sec (12.2 ft/sec)

### 3.4. Intake Deploy

- **Control Mode:** Motion Magic Position Control with Torque Current FOC
- **Axis Gear Ratio:** 20.35714:1 (32/12 × 38/16 × 40/16 × 18/14)

### 3.5. Floor Rollers (Left & Right)

- **Control Mode:** Velocity Voltage Control

### 3.6. Vertical Feeder Rollers (Left & Right)

- **Control Mode:** Velocity Voltage Control

### 3.7. Turret (Left & Right)

- **Control Mode:** Motion Magic Position Control with Torque Current FOC

### 3.8. Hood (Left & Right)

- **Control Mode:** Motion Magic Position Control with Torque Current FOC

### 3.9. Shooter (Left)

- **Control Mode:** Velocity Control with TorqueCurrent FOC
- **Axis Center X Distance from Robot Center:** -6.4 in
- **Axis Center Y Distance from Robot Center:** +6.831 in
- **Shooter Top Wheel Z Distance from Floor:** 20.5 in

### 3.10. Shooter (Right)

- **Control Mode:** Velocity Control with TorqueCurrent FOC
- **Axis Center X Distance from Robot Center:** -6.4 in
- **Axis Center Y Distance from Robot Center:** -6.831 in
- **Shooter Top Wheel Z Distance from Floor:** 20.5 in

---

## 4. Motor & Encoder Specification

- All axes utilizing **Talon FX Motor Controller** with integrated encoder
- **Encoder Ticks per Motor Rev:** 2048

---

## 5. Camera Information

### 5.1. Camera #1 (Rear Camera): Limelight 4

| Parameter | Value |
|-----------|-------|
| X Position from Robot Center | -1.098 in |
| Y Position from Robot Center | 0 in |
| Z Position from Robot Floor | 20.338 in |
| Z Rotation Angle from Robot Front | 180 deg |
| Y Rotation Angle from Ground | 20 deg |
| X Rotation Angle from Ground | TBD |

### 5.2. Camera #2 (Right Camera): Limelight 4

| Parameter | Value |
|-----------|-------|
| X Position from Robot Center | -3.132 in |
| Y Position from Robot Center | -13.179 in |
| Z Position from Robot Floor | 13.558 in |
| Z Rotation Angle from Robot Front | -90 deg |
| Y Rotation Angle from Ground | 180 deg (Rotated Upside Down) |
| X Rotation Angle from Ground | 0 deg |

### 5.3. Camera #3 (Left Camera): Limelight 4

| Parameter | Value |
|-----------|-------|
| X Position from Robot Center | -3.312 in |
| Y Position from Robot Center | 13.179 in |
| Z Position from Robot Floor | 13.558 in |
| Z Rotation Angle from Robot Front | 90 deg |
| Y Rotation Angle from Ground | 0 deg |
| X Rotation Angle from Ground | 0 deg |

### 5.4. Camera #4: Limelight 4 - DOES NOT EXIST

| Parameter | Value |
|-----------|-------|
| X Position from Robot Center | TBD |
| Y Position from Robot Center | TBD |
| Z Position from Robot Floor | TBD |
| Z Rotation Angle from Robot Front | TBD |
| Y Rotation Angle from Ground | TBD |
| X Rotation Angle from Ground | TBD |

---

## 6. Robot State and Modes Diagram

*TBD*

---

## 7. Autonomous Mode Prioritization

*TBD*

---

## Appendix 1

*TBD*

---

## Appendix 2 - Field Constants

| Field Location | Start X (in) | End X (in) | Start Y (in) | End Y (in) | Z (in) |
|----------------|-------------|-----------|-------------|-----------|--------|
| Blue Alliance Zone | 0 | 179 | 0 | 318 | 0 |
| Red Alliance Zone | 472 | 651 | 0 | 318 | 0 |
| Neutral Zone | 179 | 472 | 0 | 318 | 0 |
| Blue Hub | 182 | 182 | 159 | 159 | 79.5 |
| Red Hub | 469 | 469 | 159 | 159 | 79.5 |
| Blue Outpost Landing Zone | 80 | 80 | 0 | 72 | 0 |
| Blue Depot Landing Zone | 80 | 80 | 238 | 238 | 0 |
| Red Outpost Landing Zone | 571 | 571 | 0 | 72 | 0 |
| Red Depot Landing Zone | 571 | 571 | 238 | 238 | 0 |

---

> *Blackhawks Robotics CONFIDENTIAL*
