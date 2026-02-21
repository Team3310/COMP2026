# 3310 - Blackhawk Robotics

## Limelight Design Sheet — Practice Robot

| | |
|---|---|
| **Version:** | 2.0 |
| **Date:** | 2/21/2026 |
| **Author(s):** | Software Team |
| **Robot:** | Practice Robot |

---

## Overview

This document contains **everything** needed to wire, flash, and configure the three Limelight 4 cameras on the practice robot from scratch. Follow each section in order. When finished, all three cameras should be producing MegaTag 2 pose estimates that the robot code consumes automatically.

---

## Table of Contents

1. [Prerequisites & Downloads](#1-prerequisites--downloads)
2. [Wiring](#2-wiring)
3. [Accessing the Web Interface](#3-accessing-the-web-interface)
4. [Flashing Firmware (LLOS 2026.0)](#4-flashing-firmware-llos-20260)
5. [Focusing the Lens](#5-focusing-the-lens)
6. [Network Configuration](#6-network-configuration)
7. [Field Map Upload](#7-field-map-upload)
8. [Pipeline Configuration](#8-pipeline-configuration)
9. [Camera Pose (Robot-Space Position)](#9-camera-pose-robot-space-position)
10. [IMU Mode](#10-imu-mode)
11. [LED & Stream Settings](#11-led--stream-settings)
12. [Intrinsics Calibration](#12-intrinsics-calibration)
13. [Thermal Management & Rewind](#13-thermal-management--rewind)
14. [Verification Checklist](#14-verification-checklist)
15. [Quick Reference — Code ↔ Web UI Mapping](#15-quick-reference--code--web-ui-mapping)
16. [Troubleshooting](#16-troubleshooting)

---

## 1. Prerequisites & Downloads

Before touching the robot, collect every file you need. All files are pre-downloaded into `docs/limelight-install/` (git-ignored).

| File | What It Is | Source |
|------|-----------|--------|
| `LimelightHardwareManagerSetup2_0_6.exe` | Flash tool + device finder (Windows) | [Download](https://downloads.limelightvision.io/software/LimelightHardwareManagerSetup2_0_6.exe) |
| `FRC2026_WELDED.fmap` | 2026 AprilTag field map (welded/original fields) | [Download](https://downloads.limelightvision.io/models/FRC2026_WELDED.fmap) |
| `FRC2026_ANDYMARK.fmap` | 2026 AprilTag field map (AndyMark rebuilt fields) | [Download](https://downloads.limelightvision.io/models/FRC2026_ANDYMARK.fmap) |
| `ChArUco_CalibrationBoard.pdf` | Print at 200 × 150 mm for intrinsics calibration | [Download](https://downloads.limelightvision.io/models/calib.io_charuco_200x150_8x11_15_12_DICT_5X5.pdf) |

> **LLOS firmware image:** The Hardware Manager downloads the firmware image automatically during the flash process — you do **not** need a separate `.img` file.

> **Neural network models (optional):** No 2026-specific game-piece models have been released yet. If you want object detection later, check [downloads](https://docs.limelightvision.io/docs/resources/downloads) for new models or train your own at [tools.limelightvision.io](https://tools.limelightvision.io/).

### Install Hardware Manager

1. Run `LimelightHardwareManagerSetup2_0_6.exe` from `docs/limelight-install/`.
2. Follow the installer — it bundles all required USB drivers.
3. Launch **Limelight Hardware Manager** to verify it opens.

---

## 2. Wiring

Repeat for **each** of the three Limelight 4 cameras.

### 2a. Power

1. Run two **18–20 AWG** wires from the Limelight's **Weidmuller power port** to an open slot on the **PDP / PDH / Mini PDP**.
2. Use a **5A or 10A breaker** in that slot.
3. *(Optional but recommended)* Crimp **Weidmuller ferrules** onto the wire ends for a solid connection.
   - Ferrules: [Digi-Key](https://www.digikey.com/en/products/detail/weidm%C3%BCller/0409500000/491834) or [CTR Electronics](https://store.ctr-electronics.com/products/weidmuller-ferrule-pack-of-20?variant=43631135195309)
   - Ferrule crimper: [CTR Electronics](https://store.ctr-electronics.com/products/ferrule-crimper-by-iwiss) or [Amazon](https://www.amazon.com/Ferrule-Crimping-Hexagonal-Ferrules-Connectors/dp/B09724DCJH?th=1)

> **No VRM required.** LL4 has a built-in buck-boost converter (5–26 V input) with enhanced transient suppression. It survives battery disconnect and regenerative braking.

### 2b. Ethernet

1. Run a **Cat6 stranded** Ethernet cable from the Limelight's **RJ45 port** to the **robot radio**.
2. Add a **strain relief** (zip-tie loop or adhesive cable clamp) near the RJ45 plug.

> POE is **not** supported on LL4.

### 2c. Mounting

1. Mount the Limelight in **landscape orientation** (long edge horizontal). The internal IMU requires landscape.
2. Use **#10-32 or #10-24 × 1¼" screws** with nylock nuts through the thru-holes, or **M3 screws** into the threaded back holes.
3. Use plastic washers to preserve the anodization.

---

## 3. Accessing the Web Interface

The web UI is where all configuration happens. You have four ways to reach it:

| Method | URL | When to Use |
|--------|-----|-------------|
| **Hardware Manager** | *(scan & double-click)* | Best first-time method |
| **Hostname (mDNS)** | `http://limelight.local:5801` | After hostname is set & on robot network |
| **Static IP** | `http://10.33.10.11:5801` (example) | After static IP is configured |
| **USB (Windows)** | `http://172.28.0.1:5801` | Direct laptop → LL4 USB-C connection |

> **USB note:** Power the LL4 via Weidmuller while using USB-Ethernet — USB alone may not supply enough current.

---

## 4. Flashing Firmware (LLOS 2026.0)

Every camera must run **LLOS 2026.0** (released 1/23/2026). This version is required for `stddevs` NT key, Rewind, and the improved IMU fusion.

> ⚠️ **Flashing erases all pipelines and scripts.** Back up any custom pipelines first.

### Step-by-step

1. **Power off** the Limelight (disconnect Weidmuller power).
2. **Hold the config button** on the Limelight (small button on the side).
3. While holding the button, **plug in a USB-C cable** from your laptop to the LL4.
4. Open **Limelight Hardware Manager**.
5. Navigate to the **"Flash OS"** tab.
6. The Hardware Manager will download LLOS 2026.0 automatically. Wait for extraction to complete.
7. Click **"Refresh Device List"** and select your Limelight.
8. Click **"Flash Device"** and wait for completion (may take 2–5 minutes).
9. **Remove the USB cable** after completion.
10. Reconnect Weidmuller power and Ethernet.

Repeat for all three cameras.

---

## 5. Focusing the Lens

A properly focused lens is critical for long-range AprilTag detection.

1. Power on the robot and connect to the Limelight web UI (Section 3).
2. Click **"Ignore NT Pipeline Index"** at the top to enable manual pipeline switching.
3. Switch to **Pipeline 9**.
4. Set the pipeline type to **"Focus"**.
5. Go to the **Configuration** tab and set **stream quality to maximum**.
6. Point the camera at something with high contrast (e.g., the ChArUco calibration board, or a detailed poster).
7. **Rotate the M12 lens** to maximize the on-screen **focus score**.
8. Once maximized, apply **3–8 dots of super glue or super glue gel** around the lens barrel to lock it in place.

> After focusing, switch back to Pipeline 0 before continuing.

---

## 6. Network Configuration

Connect to each Limelight one at a time. Navigate to **Settings → Networking** in the web UI.

### 6a. Set Team Number

1. Enter `3310` in the **Team Number** field.
2. Click **"Update Team Number"**.

### 6b. Set Hostname

1. Enter the hostname exactly as shown below.
2. Click **"Set Hostname"**.

### 6c. Set Static IP

1. Change **IP Assignment** to **"Static"**.
2. Enter the IP, netmask, and gateway.
3. Click **"Update"**.
4. **Power-cycle the robot** after setting static IPs.

| Camera | Hostname | Static IP | Subnet Mask | Gateway |
|--------|----------|-----------|-------------|---------|
| Rear | `limelight-rear` | `10.33.10.11` | `255.255.255.0` | `10.33.10.1` |
| Right | `limelight-right` | `10.33.10.12` | `255.255.255.0` | `10.33.10.1` |
| Left | `limelight-left` | `10.33.10.13` | `255.255.255.0` | `10.33.10.1` |

> **Critical:** The hostname **must** match exactly (lowercase, with hyphens). The robot code uses these hostnames to address each camera on NetworkTables.

After setting the hostname, verify access at `http://<hostname>.local:5801` (e.g., `http://limelight-rear.local:5801`).

> If `limelight.local:5801` doesn't work, check that **Bonjour** is installed on Windows. Uninstall "Bonjour Print Services" if you have two Bonjour entries. Install from the [Downloads page](https://docs.limelightvision.io/docs/resources/downloads) if missing.

---

## 7. Field Map Upload

The field map tells the Limelight where every AprilTag is on the field. Without it, MegaTag localization cannot work.

1. In the web UI, go to **Settings → 3D** (or the **Field Map** section).
2. Click **"Upload Map"** (or drag-and-drop).
3. Select the correct `.fmap` file from `docs/limelight-install/`:
   - **`FRC2026_WELDED.fmap`** — for competition fields with welded/original perimeter
   - **`FRC2026_ANDYMARK.fmap`** — for practice fields with AndyMark rebuilt perimeter
4. Verify: The 3D visualizer should now show the 2026 field with wall-mounted tags.

> **Which map for our practice field?** If the field uses the original welded perimeter, use `WELDED`. If rebuilt with AndyMark components, use `ANDYMARK`. When in doubt, use `WELDED`.

> Wall-mounted tags will only display correctly in the 3D viewer with LLOS 2026.0+. This is cosmetic — localization accuracy is unaffected.

Repeat for all three cameras.

---

## 8. Pipeline Configuration

On each camera, select **Pipeline 0** and configure it as the default AprilTag pipeline.

### Step-by-step in the web UI

1. Click the **pipeline dropdown** at the top of the screen and select **Pipeline 0**.
2. In the **Input** tab:
   - **Pipeline Type:** `AprilTag`
   - **Resolution:** `1280×800` (default)
   - **LEDs:** `Off` (Force Off)
   - **Exposure:** Start at default; tune if tags are washed out under bright lights
   - **Black Level:** Leave default
   - **Sensor Gain:** Leave default; increase only if image is too dark
3. In the **AprilTag / 3D** tab:
   - **Tag Family:** `36h11`
   - **Tag Size:** `6.5 in` (165.1 mm) — verify against the 2026 game manual
   - **Detector Downscale:** `2` (default). Lower = more range but slower; 2 is a good starting point
   - **3D Mode:** `Enabled` ✅

| Setting | Value | Notes |
|---------|-------|-------|
| **Pipeline Type** | AprilTag | Required for MegaTag 2 |
| **Tag Family** | 36h11 | 2026 FRC standard |
| **Tag Size** | 6.5 in (165.1 mm) | Verify against game manual |
| **Detector Downscale** | 2 | Trade-off: range vs. speed |
| **3D Mode** | Enabled | Required for pose estimation |

> Limelight supports **10 hot-swappable pipelines** (0–9). Pipeline 0 is the default used by robot code. Pipeline 9 was used for focusing earlier.

---

## 9. Camera Pose (Robot-Space Position)

These values tell the Limelight where it is mounted relative to the robot center. Enter them in **Settings → 3D → Camera Position in Robot Space**.

### Coordinate system (Limelight web UI)

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

## 10. IMU Mode

Limelight 4 has a built-in IMU that runs at 1 kHz — much faster than the 50 Hz robot loop. To get the best MegaTag 2 accuracy, the robot code uses a **two-phase** IMU strategy recommended by the official Limelight docs:

1. **While disabled (pre-match):** IMU mode **1** — "External Seed". The LL4's internal IMU is continuously calibrated to match the gyro heading sent via `SetRobotOrientation()`.
2. **While enabled (auto / teleop):** IMU mode **4** — "Internal + External Assist". The LL4 uses its own 1 kHz IMU for frame-by-frame motion, while the robot's gyro gently corrects drift over time.

### All IMU Modes Reference

| Mode | Name | Description |
|------|------|-------------|
| 0 | External Only | Use external IMU, do not seed internal IMU |
| 1 | External Seed | Use external IMU, continuously seed internal IMU |
| 2 | Internal Only | Use internal IMU (no external input) |
| 3 | IMU Assist MT1 | Internal IMU with MegaTag 1 yaw correction |
| 4 | IMU Assist External | Internal IMU with external gyro gently correcting drift |

### Setting in Web UI

In **Settings → 3D → IMU Mode**, set the **initial** value:

| Setting | Value |
|---------|-------|
| **IMU Mode** | **External Seed (mode 1)** |

> The code automatically switches between mode 1 (disabled) and mode 4 (enabled), but setting mode 1 in the web UI ensures the internal IMU begins seeding immediately on boot.

> ⚠️ **Landscape mount required:** The LL4's internal IMU only works correctly when the camera is mounted in **landscape** orientation (long edge horizontal). Verify all three cameras are landscape-mounted.

---

## 11. LED & Stream Settings

In the **Input** tab of Pipeline 0:

| Setting | Value | Notes |
|---------|-------|-------|
| **LED Mode** | Off (Force Off) | LEDs are not needed for AprilTag detection and can distract drivers |
| **Stream Orientation** | Standard | Or flip if camera is mounted upside-down (Right camera uses Roll=180° instead) |

> **Stream Mode** can be set to "PiP Main" if the drive team wants a camera stream on the dashboard.

---

## 12. Intrinsics Calibration

Custom intrinsics calibration significantly improves pose accuracy. LLOS 2026.0 overhauled the calibration workflow.

### Prerequisites

1. **Print** the `ChArUco_CalibrationBoard.pdf` from `docs/limelight-install/` at **exactly 200 × 150 mm**.
   - Print on rigid material (foam board) or mount on a flat clipboard.
   - Verify the printed dimensions match — scaling errors ruin the calibration.

### Step-by-step (per camera)

1. In the web UI, go to a pipeline and set its type to **"Charuco Calibration Preview"**.
2. Configure the ChArUco board settings:
   - **Board Size:** 8 × 11
   - **Square Size:** 15 mm
   - **Marker Size:** 12 mm
   - **Dictionary:** 5×5
3. Point the camera at the printed board and verify **live corner detections** appear in the preview.
4. Capture **15–20 screenshots** from various angles and distances:
   - Include shots from corners, edges, close-up, and far away.
   - Fill the entire frame in some shots.
5. Go to the **Calibration** tab and click **"Calibrate"**.
6. **Download** the calibration result file.
7. **Upload** the calibration file back to the camera.
8. Verify the calibration tab header turns **bright green** (yellow = still using default calibration).

> **Reprojection error target:** < 2 pixels is good, < 1 pixel is excellent with high-quality printed targets.

Repeat for all three cameras.

---

## 13. Thermal Management & Rewind

### Thermal Throttling

LL4 generates heat. Use the `throttle_set` NT key to skip frames:

| Robot State | `throttle_set` Value | Effect |
|-------------|---------------------|--------|
| **Disabled** | 100–200 | Skips 100–200 frames between processed frames; keeps camera cool |
| **Enabled** | 0 | Full speed — processes every frame |

> The robot code should set `throttle_set` via `LimelightHelpers.setThrottle()` based on robot enable/disable state. Alternatively, switch to a **Viewfinder pipeline** while disabled to minimize processing.

### Rewind (LLOS 2026.0 — LL4 Only)

Rewind records video + targeting data to `.rwnd` files for post-match analysis. It is always-on by default but only flushes to disk on command.

- **Web UI:** After a match, click **"Record last 2m 50s"** in the web interface.
- **Code:** Use `LimelightHelpers.triggerRewindCapture("limelight-rear", 170)` (max 165s via NT).
- **Review:** Upload `.rwnd` files at [tools.limelightvision.io](https://tools.limelightvision.io/).
- **Latency cost:** 500 µs – 1 ms per frame.

---

## 14. Verification Checklist

After configuring each camera, verify the following:

- [ ] **Firmware** is LLOS 2026.0 (shown at top of web UI)
- [ ] **Hostname** appears correctly at the top of the web UI
- [ ] **IP address** is in the `10.33.10.x` range (check Settings → Networking)
- [ ] **Field map** is uploaded and the 3D visualizer shows the 2026 field
- [ ] **Pipeline 0** is an AprilTag pipeline with 36h11 family, 165.1 mm tag size
- [ ] **Camera Pose** values match Section 9 above (spot-check all 6 fields)
- [ ] **IMU Mode** is set to "External Seed" (mode 1)
- [ ] **Camera orientation** is landscape (long edge horizontal) on all three cameras
- [ ] **Lens is focused** (Pipeline 9 focus score maximized, glue applied)
- [ ] **LED Mode** is off
- [ ] **Calibration tab** header is bright green (custom calibration uploaded)
- [ ] Camera can see at least one AprilTag and the 3D visualization shows a reasonable pose
- [ ] **NetworkTables** shows entries under `limelight-rear`, `limelight-right`, `limelight-left` (check with AdvantageScope)
- [ ] `tv` flips to `1` when a tag is in view
- [ ] `botpose_orb_wpiblue` array is non-zero when a tag is in view (MegaTag 2 output)
- [ ] `stddevs` array is populated (12-element double array)

---

## 15. Quick Reference — Code ↔ Web UI Mapping

| Code Constant / Call | Value | Where It Must Match in Web UI |
|---------------------|-------|-------------------------------|
| `VisionConstants.kLimelightRear` | `"limelight-rear"` | Hostname in Settings |
| `VisionConstants.kLimelightRight` | `"limelight-right"` | Hostname in Settings |
| `VisionConstants.kLimelightLeft` | `"limelight-left"` | Hostname in Settings |
| `kRearForwardM` / etc. | See Section 9 | Camera Pose in 3D tab (entered in inches) |
| `SetIMUMode(name, 1)` / `SetIMUMode(name, 4)` | Mode 1 (disabled) / Mode 4 (enabled) | IMU Mode in 3D tab (set to 1 initially) |
| `SetRobotOrientation()` | Called every loop | Feeds gyro yaw for both seeding and MT2 |
| `getBotPoseEstimate_wpiBlue_MegaTag2()` | reads `botpose_orb_wpiblue` | Pipeline must be AprilTag with 3D enabled |
| `getLimelightNTDoubleArray(name, "stddevs")` | 12-element array | Published automatically by LLOS ≥ 2026.0 |

---

## 16. Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| Camera not visible on network | Wrong IP / hostname, or radio not connected | Re-flash via USB, check hostname and static IP. Reset IP by holding config button 10 sec after boot |
| `limelight.local:5801` doesn't resolve | Bonjour not installed | Install Bonjour; uninstall "Bonjour Print Services" if two entries exist |
| `tv` always 0 | Pipeline not set to AprilTag, or no tags in view | Switch to AprilTag pipeline, verify 36h11 family, point at a tag |
| Tags detected but no 3D pose | 3D mode disabled or no field map | Enable 3D mode; upload `.fmap` file |
| Pose estimate is wildly wrong | Camera pose values are incorrect | Double-check all 6 fields in Section 9 for each camera |
| Pose estimate works but drifts | IMU mode incorrect or not seeded | Verify mode 1 in web UI; ensure robot was disabled ≥ 30 sec to seed before enabling |
| `stddevs` array is empty or all zeros | LLOS version too old | Update to LLOS 2026.0+ |
| Robot code logs `stddevs_missing` | Camera offline or LLOS too old | Check Ethernet, update firmware |
| Robot code logs `stddevs_zero` | No valid solve yet | Ensure tags are visible and 3D mode is on |
| Camera overheating / throttling | Not using throttle while disabled | Set `throttle_set` to 100–200 while disabled |
| Focus score low / tags not detected at range | Lens not focused | Re-run focus procedure (Section 5), re-glue lens |
| Calibration tab header is yellow | Using default calibration | Run intrinsics calibration (Section 12) |

---

## Appendix: Downloaded Files Inventory

All files are in `docs/limelight-install/` and are **git-ignored**.

```
docs/limelight-install/
├── .gitignore                              ← keeps binaries out of git
├── LimelightHardwareManagerSetup2_0_6.exe  ← Windows flash tool + device finder
├── FRC2026_WELDED.fmap                     ← 2026 field map (welded perimeter)
├── FRC2026_ANDYMARK.fmap                   ← 2026 field map (AndyMark perimeter)
└── ChArUco_CalibrationBoard.pdf            ← Print at 200×150mm for calibration
```

**Re-download links** (if files are missing):
- Hardware Manager: <https://downloads.limelightvision.io/software/LimelightHardwareManagerSetup2_0_6.exe>
- WELDED map: <https://downloads.limelightvision.io/models/FRC2026_WELDED.fmap>
- ANDYMARK map: <https://downloads.limelightvision.io/models/FRC2026_ANDYMARK.fmap>
- Calibration board: <https://downloads.limelightvision.io/models/calib.io_charuco_200x150_8x11_15_12_DICT_5X5.pdf>
- All downloads: <https://docs.limelightvision.io/docs/resources/downloads>

---

> *Blackhawks Robotics CONFIDENTIAL*
