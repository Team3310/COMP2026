# 3310 - Blackhawk Robotics

## Limelight Design Sheet — Practice Robot

| | |
|---|---|
| **Version:** | 1.0 |
| **Date:** | 2/21/2026 |
| **Author(s):** | Software Team |
| **Robot:** | Practice Robot |

---

## Overview

This document contains **everything** needed to configure the three Limelight 4 cameras on the practice robot. Follow each section in order. When finished, all three cameras should be producing MegaTag 2 pose estimates that the robot code consumes automatically.

> **Prerequisite:** Limelight OS (LLOS) **2026.0 or later** must be installed on every camera. Update via the [Limelight firmware page](https://limelightvision.io/pages/downloads) before proceeding.

---

## 1. Network Configuration

Connect to each Limelight one at a time (USB or direct Ethernet) and set the following in **Settings → Networking**.

| Camera | Hostname | Static IP | Subnet Mask | Gateway | Team # |
|--------|----------|-----------|-------------|---------|--------|
| Rear | `limelight-rear` | `10.33.10.11` | `255.255.255.0` | `10.33.10.1` | `3310` |
| Right | `limelight-right` | `10.33.10.12` | `255.255.255.0` | `10.33.10.1` | `3310` |
| Left | `limelight-left` | `10.33.10.13` | `255.255.255.0` | `10.33.10.1` | `3310` |

> **Important:** The hostname **must** match exactly (lowercase, with hyphens). The robot code uses these hostnames to address each camera on NetworkTables.

After setting the hostname, the camera's web UI will be accessible at `http://<hostname>.local:5801` (e.g., `http://limelight-rear.local:5801`).

---

## 2. Pipeline Configuration

On each camera, create (or select) an **AprilTag** pipeline and set it as the default (Pipeline 0).

| Setting | Value | Notes |
|---------|-------|-------|
| **Pipeline Type** | AprilTag | Required for MegaTag 2 |
| **Tag Family** | 36h11 | 2026 FRC standard |
| **Tag Size** | 6.5 in (165.1 mm) | 2026 FRC tag size — verify against game manual |
| **Detector Downscale** | 2 (default) | Lower = more range but slower. 2 is a good starting point |
| **3D Mode** | Enabled | Required for pose estimation |

---

## 3. Camera Pose (Robot-Space Position)

These values tell the Limelight where it is mounted relative to the robot center. Enter them in **Settings → 3D → Camera Position in Robot Space**.

The Limelight web UI uses the following coordinate system:
- **Forward (X):** Positive = toward robot front
- **Side (Y):** Positive = toward robot left
- **Up (Z):** Positive = up from floor

### Camera #1 — Rear (`limelight-rear`)

| Field in Web UI | Value | Unit |
|-----------------|-------|------|
| Forward | -1.098 | inches |
| Side | 0.000 | inches |
| Up | 20.338 | inches |
| Roll (X rotation) | 0.0 | degrees |
| Pitch (Y rotation) | 20.0 | degrees |
| Yaw (Z rotation) | 180.0 | degrees |

> Rear-facing camera. Yaw = 180° means it points straight backward.

### Camera #2 — Right (`limelight-right`)

| Field in Web UI | Value | Unit |
|-----------------|-------|------|
| Forward | -3.132 | inches |
| Side | -13.179 | inches |
| Up | 13.558 | inches |
| Roll (X rotation) | 180.0 | degrees |
| Pitch (Y rotation) | 0.0 | degrees |
| Yaw (Z rotation) | -90.0 | degrees |

> **Mounted upside-down** — Roll = 180° accounts for this. Yaw = -90° means it points to the robot's right side.

### Camera #3 — Left (`limelight-left`)

| Field in Web UI | Value | Unit |
|-----------------|-------|------|
| Forward | -3.312 | inches |
| Side | 13.179 | inches |
| Up | 13.558 | inches |
| Roll (X rotation) | 0.0 | degrees |
| Pitch (Y rotation) | 0.0 | degrees |
| Yaw (Z rotation) | 90.0 | degrees |

> Yaw = 90° means it points to the robot's left side.

---

## 4. IMU Mode

The robot code sends our NavX/Pigeon gyro heading to each Limelight every loop via `SetRobotOrientation()`. For this to work, each camera must be set to accept **external** orientation data.

In **Settings → 3D → IMU Mode**:

| Setting | Value |
|---------|-------|
| **IMU Mode** | **Use External (mode 0)** |

> The code also pushes this setting automatically every ~5 seconds, but setting it in the web UI ensures it's correct immediately on boot.

---

## 5. LED & Stream Settings

| Setting | Value | Notes |
|---------|-------|-------|
| **LED Mode** | Off (Force Off) | LEDs are not needed for AprilTag detection and can distract drivers |
| **Stream Mode** | Standard | Or "PiP Main" if the drive team wants a camera stream on the dashboard |

---

## 6. Verification Checklist

After configuring each camera, verify the following:

- [ ] **Hostname** appears correctly at the top of the web UI
- [ ] **IP address** is in the `10.33.10.x` range (check Settings → Networking)
- [ ] **Pipeline 0** is an AprilTag pipeline with 36h11 family
- [ ] **Camera Pose** values match Section 3 above (spot-check all 6 fields)
- [ ] **IMU Mode** is set to "Use External" (mode 0)
- [ ] **LED Mode** is off
- [ ] Camera can see at least one AprilTag and the 3D visualization shows a reasonable pose
- [ ] **NetworkTables** shows entries under `limelight-rear`, `limelight-right`, `limelight-left` (check with OutlineViewer or AdvantageScope)
- [ ] `tv` flips to `1` when a tag is in view
- [ ] `botpose_orb_wpiblue` array is non-zero when a tag is in view (this is the MegaTag 2 output)

---

## 7. Quick Reference — Code ↔ Web UI Mapping

| Code Constant | Value | Where It Must Match |
|---------------|-------|---------------------|
| `VisionConstants.kLimelightRear` | `"limelight-rear"` | Hostname in LL web UI |
| `VisionConstants.kLimelightRight` | `"limelight-right"` | Hostname in LL web UI |
| `VisionConstants.kLimelightLeft` | `"limelight-left"` | Hostname in LL web UI |
| `kRearForwardM` / etc. | See Section 3 | Camera Pose in 3D tab (entered in inches) |
| `SetIMUMode(name, 0)` | Mode 0 | IMU Mode in 3D tab |
| `SetRobotOrientation()` | Called every loop | Requires IMU Mode = 0 to take effect |
| `getBotPoseEstimate_wpiBlue_MegaTag2()` | reads `botpose_orb_wpiblue` | Pipeline must be AprilTag with 3D enabled |
| `getLimelightNTDoubleArray(name, "stddevs")` | 12-element array | Published automatically by LLOS ≥ 2026.0 |

---

## 8. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Camera not visible on network | Wrong IP / hostname | Re-flash via USB, check hostname and static IP |
| `tv` always 0 | Pipeline not set to AprilTag, or no tags in view | Switch to AprilTag pipeline, point at a tag |
| Pose estimate is wildly wrong | Camera pose values are incorrect | Double-check all 6 fields in Section 3 |
| Pose estimate works but drifts | IMU mode not set to external | Set IMU Mode to 0 in web UI |
| `stddevs` array is empty or all zeros | LLOS version too old | Update to LLOS 2026.0+ |
| Robot code logs `stddevs_missing` | Camera offline or LLOS too old | Check network, update firmware |
| Robot code logs `stddevs_zero` | No valid solve yet | Ensure tags are visible and 3D mode is on |

---

> *Blackhawks Robotics CONFIDENTIAL*
