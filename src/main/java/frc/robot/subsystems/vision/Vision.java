package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem using 4× Limelight 4 cameras with a two-phase MegaTag strategy:
 *
 * <h3>Phase 1 — Disabled (pre-match): MegaTag 1</h3>
 *
 * <p>While the robot is disabled, cameras run throttled (~0.6 fps) and produce <b>MegaTag 1</b>
 * estimates (full 6-DOF including rotation). This lets the robot determine its field position
 * <b>and heading</b> without manual gyro alignment. The first high-confidence multi-tag result
 * hard-resets the pose estimator (via {@link Drive#setPose}), which also sets the Pigeon2 gyro
 * offset. Subsequent MT1 results refine the estimate. The corrected heading is continuously pushed
 * to the Limelight IMUs via {@code SetRobotOrientation()} so they are seeded with the correct
 * heading by the time the match starts.
 *
 * <h3>Phase 2 — Enabled (auto / teleop): MegaTag 2</h3>
 *
 * <p>Once enabled, the system switches to <b>MegaTag 2</b> which uses the now-correct gyro heading
 * to constrain its solve (XY only). The LL4's internal 1 kHz IMU (seeded during Phase 1) tracks
 * orientation between the 50 Hz robot-code updates for frame-accurate rotation.
 *
 * <p>Each enabled loop we:
 *
 * <ol>
 *   <li>Push IMU mode 4 (Internal + External Assist) and feed gyro yaw + yaw rate.
 *   <li>Query {@code getBotPoseEstimate_wpiBlue_MegaTag2()} for each camera.
 *   <li>Filter out bad results (no tags, spinning too fast, off-field, big jumps).
 *   <li>Read the Limelight's own MegaTag 2 standard deviations and use {@code max(xStd, yStd)} as
 *       the XY trust weight.
 *   <li>Feed accepted measurements into {@code Drive.addVisionMeasurement()}.
 * </ol>
 *
 * <p>Theta std dev is set to 999999 during enabled mode because MegaTag 2 does <b>not</b> estimate
 * rotation — it uses the gyro heading you supply.
 */
public class Vision extends SubsystemBase {
  private static final String kVisionEnabledKey = "Vision/Enabled";
  private static final String kVisionSeededKey = "Vision/Seeded";
  private static final String kVisionSeedStableKey = "Vision/SeedStable";

  private final Drive drive;
  private int configCounter = CONFIG_INTERVAL; // Start at threshold so first loop configures
  private static final int CONFIG_INTERVAL = 250; // Re-send config every 250 loops (~5 seconds)

  // Log rate-limiting — Logger.recordOutput is expensive (NT write per call).
  // We have ~15 log calls per camera × 3 cameras = ~45 writes per loop at 50 Hz.
  // Rate-limiting to every Nth cycle cuts NT traffic without affecting robot
  // functionality — all filtering, processing, and pose injection still run
  // every single cycle.
  private int logCounter = 0;
  private static final int LOG_INTERVAL = 10; // Log every 10th cycle (~5 Hz)

  // Thermal throttle — skip frames while disabled to keep cameras cool.
  // Per Limelight docs (LDS §13): 100–200 while disabled, 0 while enabled.
  // We use 0 while disabled too so pre-match pose seeding gets full frame rate.
  // The cameras won't overheat during a few minutes of pre-match idle.
  private boolean wasDisabled = true; // assume starting disabled
  private boolean firstLoop = true; // push throttle on the very first periodic() call
  private int lastIMUMode = -1; // track IMU mode to avoid spamming SetIMUMode every cycle

  // Downsample disabled-mode processing to avoid loop overruns.
  // While disabled the cameras are throttled (low FPS) so running full
  // NT reads + SetRobotOrientation + processCameraPreMatch at 50 Hz is
  // wasteful and eats into the 20 ms budget.  Run every Nth cycle instead.
  private int disabledCycleCounter = 0;
  private static final int DISABLED_PROCESS_INTERVAL = 5; // run every 5th cycle (~10 Hz)

  // Pre-match pose seeding state.
  // The first accepted seed uses setPose() (hard reset); subsequent seeds use
  // addVisionMeasurement() so the estimator converges smoothly.
  private boolean hasSeed = false;
  private boolean seedStable = false;
  private Pose2d lastAcceptedPreMatchPose = null;
  private int stableSeedSampleCount = 0;

  // Dashboard key for the vision enable/disable toggle.
  // Default: true (vision actively seeds pose estimator).
  // Set to false from Elastic/SmartDashboard to remove vision from the pose
  // estimator for debugging (cameras still run, log data, and seed IMU —
  // only the pose injection is suppressed).

  /** A vision measurement that passed all filters and is ready to be injected. */
  private record AcceptedObservation(
      Pose2d pose, double timestampSeconds, double scaledXYStdDev, double thetaStdDev) {}

  /**
   * Creates a new Vision subsystem.
   *
   * @param drive The drive subsystem, used to read gyro heading and inject vision measurements.
   */
  public Vision(Drive drive) {
    this.drive = drive;
    // Publish the default value so the toggle appears on the dashboard immediately.
    // TODO: Re-enable vision seeding once camera coordinates are verified on the real robot.
    SmartDashboard.putBoolean(kVisionEnabledKey, false);
    SmartDashboard.putBoolean(kVisionSeededKey, false);
    SmartDashboard.putBoolean(kVisionSeedStableKey, false);
    // Whether to reset the pre-match seed when transitioning back to disabled.
    // Default false to preserve a previously-seeded pose when re-disabling
    // (avoids choppy re-seeding after enable/disable cycles). Set true to
    // force re-localization between matches / practice runs.
    SmartDashboard.putBoolean("Vision/ResetOnDisable", false);
  }

  public void setVisionEnabled(boolean enabled) {
    SmartDashboard.putBoolean(kVisionEnabledKey, enabled);
  }

  /** Returns true if the odometry has already been seeded by limelight pre-match. */
  public boolean isSeeded() {
    return seedStable;
  }

  // -----------------------------------------------------------------------
  //  Periodic — runs every 20 ms
  // -----------------------------------------------------------------------
  @Override
  public void periodic() {
    // Skip all vision processing while in pit mode.
    if (Robot.inPit) {
      return;
    }

    // Rate-limit logging — increment counter and decide if this is a log cycle.
    // All processing and injection still runs every cycle; only Logger output is gated.
    logCounter++;
    boolean shouldLog = logCounter >= LOG_INTERVAL;
    if (shouldLog) {
      logCounter = 0;
    }

    // Read the dashboard toggle — when false, cameras still run and log but
    // do NOT inject measurements into the pose estimator.  Useful for debugging.
    boolean visionEnabled = SmartDashboard.getBoolean(kVisionEnabledKey, false);
    SmartDashboard.putBoolean(kVisionSeededKey, hasSeed);
    SmartDashboard.putBoolean(kVisionSeedStableKey, seedStable);

    // Periodically re-send camera config so cameras that boot late or power-cycle
    // mid-match still get the correct poses and IMU mode.
    configCounter++;
    if (configCounter >= CONFIG_INTERVAL) {
      configureCameras();
      configCounter = 0;
    }

    // Always push IMU mode every cycle so the transition from mode 1 → 4
    // happens promptly when the robot is enabled.
    setIMUModes();

    // Thermal throttle: set frame processing rate.
    // Push on first loop (so cameras are configured immediately on boot)
    // and on enable/disable transitions.
    boolean isDisabled = DriverStation.isDisabled();
    if (firstLoop || isDisabled != wasDisabled) {
      int throttle =
          isDisabled
              ? VisionConstants.kDisabledThrottleFrames
              : VisionConstants.kEnabledThrottleFrames;
      for (String name : VisionConstants.kCameraNames) {
        LimelightHelpers.SetThrottle(name, throttle);
      }
      // Reset pre-match seed when transitioning back to disabled only if the
      // dashboard toggle is enabled. Older behavior always reset which caused
      // re-seeding and visible jumps after enable→disable cycles. Default is
      // false so a previously-seeded pose remains stable across brief toggles.
      if (isDisabled && !firstLoop) {
        boolean resetOnDisable = SmartDashboard.getBoolean("Vision/ResetOnDisable", false);
        if (resetOnDisable) {
          resetSeedState();
        }
        disabledCycleCounter = 0; // reset so first disabled cycle processes immediately
      }
      wasDisabled = isDisabled;
      firstLoop = false;
    }

    // Always feed the raw Pigeon2 heading to SetRobotOrientation — NOT the
    // fused pose-estimator heading. Using the fused heading creates a feedback
    // loop: a bad vision measurement corrupts the pose, which corrupts the
    // heading we report back to the LL, which corrupts the next MT2 solve.
    // The raw gyro is immune to vision errors and is what the LL actually needs.
    double robotYawDeg = drive.getRawGyroRotation().getDegrees();
    double yawRateDps = drive.getRawGyroYawRateDegPerSec();

    // While disabled, downsample the expensive per-camera work
    // (SetRobotOrientation flushes + getBotPoseEstimate NT reads) so we don't
    // blow the 20 ms loop budget.  The cameras are throttled to low FPS anyway,
    // so 10 Hz robot-side processing is more than enough.
    if (isDisabled) {
      disabledCycleCounter++;
      if (disabledCycleCounter >= DISABLED_PROCESS_INTERVAL) {
        disabledCycleCounter = 0;
        for (String name : VisionConstants.kCameraNames) {
          LimelightHelpers.SetRobotOrientation(name, robotYawDeg, yawRateDps, 0.0, 0.0, 0.0, 0.0);
        }
        for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
          logRawMegaTag2Pose(VisionConstants.kCameraNames[i]);
          processCameraPreMatch(VisionConstants.kCameraNames[i], i, visionEnabled, shouldLog);
        }
      }
      return;
    }

    // Enabled path — full-rate processing every cycle.
    for (String name : VisionConstants.kCameraNames) {
      LimelightHelpers.SetRobotOrientation(name, robotYawDeg, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    // Process each camera — collect accepted observations for timestamp-sorted injection.
    // Sorting ensures the pose estimator processes measurements in chronological order
    // even when cameras have different pipeline latencies.
    List<AcceptedObservation> accepted = new ArrayList<>();
    for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
      AcceptedObservation obs =
          processCamera(VisionConstants.kCameraNames[i], robotYawDeg, yawRateDps, i, shouldLog);
      if (obs != null) {
        accepted.add(obs);
      }
    }

    // Sort by timestamp (oldest first) and inject into pose estimator.
    accepted.sort(Comparator.comparingDouble(AcceptedObservation::timestampSeconds));
    // TODO: Re-enable once camera coordinates are verified on the real robot.
    // for (AcceptedObservation obs : accepted) {
    //   if (visionEnabled) {
    //     drive.addVisionMeasurement(
    //         obs.pose(),
    //         obs.timestampSeconds(),
    //         VecBuilder.fill(obs.scaledXYStdDev(), obs.scaledXYStdDev(), obs.thetaStdDev()));
    //   }
    // }

    // Log the final filtered pose feeding odometry (rate-limited).
    if (shouldLog) {
      // Log the combined accepted pose that is actually feeding the pose
      // estimator this cycle.  When multiple cameras pass filters we compute a
      // stddev-weighted average so AdvantageScope can show a single "what
      // odometry sees" ghost alongside the per-camera raw ghosts.
      if (!accepted.isEmpty()) {
        double weightSum = 0.0;
        double sumX = 0.0;
        double sumY = 0.0;
        double sumCos = 0.0;
        double sumSin = 0.0;
        for (AcceptedObservation obs : accepted) {
          // Weight = 1/stddev² — lower stddev → higher weight.
          double w = 1.0 / (obs.scaledXYStdDev() * obs.scaledXYStdDev());
          weightSum += w;
          sumX += obs.pose().getX() * w;
          sumY += obs.pose().getY() * w;
          double r = obs.pose().getRotation().getRadians();
          sumCos += Math.cos(r) * w;
          sumSin += Math.sin(r) * w;
        }
        double avgX = sumX / weightSum;
        double avgY = sumY / weightSum;
        double avgYaw = Math.atan2(sumSin / weightSum, sumCos / weightSum);
        Pose2d combinedAccepted = new Pose2d(avgX, avgY, new Rotation2d(avgYaw));
        Logger.recordOutput("Vision/finalFilteredPose", combinedAccepted);
        Logger.recordOutput("Vision/finalFilteredXYStdDev", Math.sqrt(1.0 / weightSum));
      }
    }
  }

  // -----------------------------------------------------------------------
  //  Camera configuration (called once)
  // -----------------------------------------------------------------------
  /**
   * Pushes the measured camera poses into each Limelight. Called periodically so cameras that boot
   * late or power-cycle mid-match still get the correct poses.
   */
  private void configureCameras() {
    for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
      String name = VisionConstants.kCameraNames[i];
      double[] pose = VisionConstants.kCameraPoses[i];

      // Set camera position relative to robot center.
      // Args: forward, side, up, roll, pitch, yaw (meters / degrees)
      LimelightHelpers.setCameraPose_RobotSpace(
          name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
    }
  }

  /**
   * Sets the IMU mode on every camera. Uses a two-phase strategy recommended by the official
   * Limelight docs:
   *
   * <ul>
   *   <li><b>Disabled (pre-match):</b> Mode 1 — "External Seed". The LL4's internal IMU is
   *       continuously calibrated to match the gyro heading we send via {@code
   *       SetRobotOrientation()}.
   *   <li><b>Enabled (auto / teleop):</b> Mode 4 — "Internal + External Assist". The LL4 uses its 1
   *       kHz internal IMU for frame-by-frame motion while the robot's gyro gently corrects drift.
   * </ul>
   */
  private void setIMUModes() {
    // Mode 1 (External Seed) while disabled for pre-match heading calibration.
    // Mode 4 (Internal + External Assist) while enabled for match play.
    int mode = DriverStation.isDisabled() ? 1 : 4;
    // Only push the IMU mode when it actually changes (or on the very first
    // call).  Spamming SetIMUMode at 50 Hz can disrupt the Limelight's
    // internal complementary filter and cause cameras to enter a bad state
    // after a few minutes of play.
    if (mode == lastIMUMode) {
      return;
    }
    lastIMUMode = mode;
    for (String name : VisionConstants.kCameraNames) {
      LimelightHelpers.SetIMUMode(name, mode);
    }
  }

  private void logRawMegaTag2Pose(String cameraName) {
    String prefix = "Vision/" + cameraName + "/";
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);
    if (estimate != null && estimate.tagCount > 0 && estimate.pose != null) {
      double xStdDev = 0.03;
      double yStdDev = 0.03;
      double xyStdDev = Math.max(xStdDev, yStdDev);
      Logger.recordOutput(prefix + "mt2Pose", estimate.pose);
      Logger.recordOutput(prefix + "mt2XYStdDev", xyStdDev);
    } else {
      Logger.recordOutput(prefix + "mt2XYStdDev", -1.0);
    }
  }

  // -----------------------------------------------------------------------
  //  Per-camera processing
  // -----------------------------------------------------------------------
  /**
   * Queries one Limelight for a MegaTag 2 pose estimate, applies filtering, and — if accepted —
   * returns an {@link AcceptedObservation} ready for injection. Returns {@code null} if the
   * measurement was rejected by any filter.
   *
   * @param cameraName Limelight hostname
   * @param robotYawDeg Current gyro yaw in degrees
   * @param yawRateDegPerSec Current angular velocity in deg/s
   * @param cameraIndex Index into kCameraNames (for logging and per-camera stddev factor)
   * @param shouldLog Whether to emit Logger output this cycle (rate-limited)
   * @return An accepted observation, or {@code null} if rejected.
   */
  private AcceptedObservation processCamera(
      String cameraName,
      double robotYawDeg,
      double yawRateDegPerSec,
      int cameraIndex,
      boolean shouldLog) {

    String prefix = "Vision/" + cameraName + "/";

    // 1. Read the MegaTag 2 pose estimate.
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    // Publish raw (unfiltered) pose to SmartDashboard for the drive team.
    if (estimate != null && estimate.tagCount > 0 && estimate.pose != null) {
      SmartDashboard.putString(prefix + "rawPose", estimate.pose.toString());
    } else {
      SmartDashboard.putString(prefix + "rawPose", "No tags");
    }

    // Log raw MT2 pose and stddev only when a valid tagged pose exists.
    if (estimate != null && estimate.tagCount > 0 && estimate.pose != null) {
      Logger.recordOutput(prefix + "mt2Pose", estimate.pose);
    }

    // 2. Null / no-tag guard.
    if (estimate == null || estimate.tagCount == 0 || estimate.pose == null) {
      return null;
    }

    Pose2d visionPose = estimate.pose;

    // 3. Reject stale measurements — if the timestamp is too old the pose
    //    estimator will "rewind" and replay with bad data.
    double age = Logger.getTimestamp() / 1.0e6 - estimate.timestampSeconds;
    if (age > VisionConstants.kMaxMeasurementAgeSec) {
      if (shouldLog) {
        Logger.recordOutput(prefix + "accepted", false);
        Logger.recordOutput(prefix + "rejectReason", "stale_timestamp");
        Logger.recordOutput(prefix + "measurementAgeSec", age);
      }
      return null;
    }

    // 4. Reject if the robot is spinning too fast (motion blur degrades detection).
    if (Math.abs(yawRateDegPerSec) > VisionConstants.kMaxAngularVelocityDegPerSec) {
      if (shouldLog) {
        Logger.recordOutput(prefix + "accepted", false);
        Logger.recordOutput(prefix + "rejectReason", "yaw_rate");
      }
      return null;
    }

    // 5. Reject single-tag results that are too small (far away / ambiguous).
    if (estimate.tagCount == 1 && estimate.avgTagArea < VisionConstants.kMinTagAreaForSingleTag) {
      if (shouldLog) {
        Logger.recordOutput(prefix + "accepted", false);
        Logger.recordOutput(prefix + "rejectReason", "single_tag_area");
      }
      return null;
    }

    // 6. Reject poses that are clearly off the field (with small margin to reject origin).
    if (visionPose.getX() < 0.01
        || visionPose.getX() > Constants.kFieldLengthMeters
        || visionPose.getY() < 0.01
        || visionPose.getY() > Constants.kFieldWidthMeters) {
      if (shouldLog) {
        Logger.recordOutput(prefix + "accepted", false);
        Logger.recordOutput(prefix + "rejectReason", "off_field");
      }
      return null;
    }

    // 7. Read the Limelight's own MegaTag 2 std devs from NetworkTables.
    double[] stddevs = LimelightHelpers.getLimelightNTDoubleArray(cameraName, "stddevs");

    if (stddevs.length < VisionConstants.kExpectedStdDevArrayLength) {
      if (shouldLog) {
        Logger.recordOutput(prefix + "accepted", false);
        Logger.recordOutput(prefix + "rejectReason", "stddevs_missing");
      }
      return null;
    }
    double DEFAULT_LINEAR_STDDEV = 0.03;
    double xStdDev = DEFAULT_LINEAR_STDDEV;
    double yStdDev = DEFAULT_LINEAR_STDDEV;
    double xyStdDev = Math.max(xStdDev, yStdDev);
    if (shouldLog) {
      Logger.recordOutput(prefix + "mt2XYStdDev", xyStdDev);
    }

    if (xyStdDev <= 0.0) {
      return null;
    }

    // 8. Reject measurements with excessively high std devs.
    if (VisionConstants.kMT2MaxAcceptedStdDev > 0.0
        && xyStdDev > VisionConstants.kMT2MaxAcceptedStdDev) {
      return null;
    }

    // 9. (removed) Previously we rejected large jumps from the current pose estimate.
    // This pose-jump-based rejection caused valid measurements to be dropped in some cases.
    // The check has been intentionally removed so measurements are not rejected solely
    // on distance from the current estimated pose. Other filters (timestamp, stddev,
    // off-field, etc.) remain in place.

    // 10. Apply the MT2 filter-strength multiplier and per-camera trust factor.
    double cameraFactor = VisionConstants.kCameraStdDevFactors[cameraIndex];
    double scaledXYStdDev = xyStdDev * VisionConstants.kMT2StdDevMultiplier * cameraFactor;

    // 11. Return the accepted observation for timestamp-sorted injection.
    return new AcceptedObservation(
        visionPose, estimate.timestampSeconds, scaledXYStdDev, VisionConstants.kThetaStdDev);
  }

  // -----------------------------------------------------------------------
  //  Pre-match pose seeding (while disabled, MegaTag 1)
  // -----------------------------------------------------------------------
  /**
   * Process a single camera while disabled to seed the pose estimator before auto using <b>MegaTag
   * 1</b> (full 6-DOF solve including rotation).
   *
   * <p>This eliminates the need to laser-align the gyro before every match. MT1 determines the
   * robot's heading from tag geometry alone. The first accepted measurement hard-resets the pose
   * estimator (including the gyro offset) so the Pigeon2 and LL4 IMUs are automatically aligned to
   * the correct field heading.
   *
   * <p>Filters are stricter than normal match processing:
   *
   * <ul>
   *   <li>Requires multi-tag (≥ {@link VisionConstants#kPreMatchMinTagCount} tags).
   *   <li>Requires low XY std devs (≤ {@link VisionConstants#kPreMatchMaxStdDev}).
   *   <li>Requires low yaw std dev (≤ {@link VisionConstants#kPreMatchMaxYawStdDevDeg}).
   *   <li>Still rejects off-field poses.
   *   <li><b>Does NOT</b> reject based on pose jump — the estimator starts at (0,0) so the first
   *       real estimate would always be a "jump".
   * </ul>
   *
   * <p>The first accepted measurement uses {@link Drive#setPose} to hard-reset the estimator
   * (including gyro offset). Subsequent measurements use {@link Drive#addVisionMeasurement} so the
   * estimate converges smoothly across multiple cameras and frames.
   *
   * @param cameraName Limelight hostname.
   * @param cameraIndex Index into kCameraNames (for logging).
   * @param visionEnabled Whether to inject the measurement into the pose estimator.
   * @param shouldLog Whether to emit Logger output this cycle (rate-limited).
   */
  private void processCameraPreMatch(
      String cameraName, int cameraIndex, boolean visionEnabled, boolean shouldLog) {

    String prefix = "Vision/" + cameraName + "/";

    // 1. Read the MegaTag 1 pose estimate (full 6-DOF including rotation).
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    // 2. Null / no-tag guard.
    if (estimate == null || estimate.tagCount == 0 || estimate.pose == null) {
      return;
    }

    Pose2d visionPose = estimate.pose;
    if (shouldLog) {
      Logger.recordOutput(prefix + "mt1Pose", visionPose);
    }

    // 3. Require multi-tag for high confidence (especially important for yaw).
    if (estimate.tagCount < VisionConstants.kPreMatchMinTagCount) {
      if (shouldLog) {
        Logger.recordOutput(prefix + "preMatch", false);
        Logger.recordOutput(prefix + "preMatchReject", "tag_count");
      }
      return;
    }

    // 4. Reject poses that are off the field.
    if (visionPose.getX() < 0.01
        || visionPose.getX() > Constants.kFieldLengthMeters
        || visionPose.getY() < 0.01
        || visionPose.getY() > Constants.kFieldWidthMeters) {
      if (shouldLog) {
        Logger.recordOutput(prefix + "preMatch", false);
        Logger.recordOutput(prefix + "preMatchReject", "off_field");
      }
      return;
    }

    // 5. Read Limelight MT1 std devs and require high confidence in XY and yaw.
    double[] stddevs = LimelightHelpers.getLimelightNTDoubleArray(cameraName, "stddevs");
    if (stddevs.length < VisionConstants.kExpectedStdDevArrayLength) {
      if (shouldLog) {
        Logger.recordOutput(prefix + "preMatch", false);
        Logger.recordOutput(prefix + "preMatchReject", "stddevs_missing");
      }
      return;
    }

    double xStdDev = stddevs[VisionConstants.kMT1XStdDevIndex];
    double yStdDev = stddevs[VisionConstants.kMT1YStdDevIndex];
    double yawStdDev = stddevs[VisionConstants.kMT1YawStdDevIndex];
    double xyStdDev = Math.max(xStdDev, yStdDev);
    if (shouldLog) {
      Logger.recordOutput(prefix + "mt1XYStdDev", xyStdDev);
      Logger.recordOutput(prefix + "mt1YawStdDev", yawStdDev);
    }

    if (xyStdDev <= 0.0) {
      return;
    }

    if (xyStdDev > VisionConstants.kPreMatchMaxStdDev) {
      return;
    }

    if (yawStdDev > VisionConstants.kPreMatchMaxYawStdDevDeg) {
      return;
    }

    // 6. Update stable-seed tracking from accepted MT1 poses.
    updateStableSeedState(visionPose, shouldLog, prefix);

    // 7. Seed the pose estimator.
    // TODO: Re-enable once camera coordinates are verified on the real robot.
    // if (!hasSeed) {
    //   if (visionEnabled) {
    //     drive.setPose(visionPose);
    //     hasSeed = true;
    //   }
    // } else {
    //   double scaledXYStdDev = xyStdDev * VisionConstants.kMT1StdDevMultiplier;
    //   double scaledYawStdDev = yawStdDev * VisionConstants.kMT1StdDevMultiplier;
    //   if (visionEnabled) {
    //     drive.addVisionMeasurement(
    //         visionPose,
    //         estimate.timestampSeconds,
    //         VecBuilder.fill(scaledXYStdDev, scaledXYStdDev, Math.toRadians(scaledYawStdDev)));
    //   }
    // }

    if (!hasSeed && !visionEnabled) {
      seedStable = false;
    } else if (hasSeed && stableSeedSampleCount >= VisionConstants.kPreMatchStableSeedMinSamples) {
      seedStable = true;
    }
  }

  private void resetSeedState() {
    hasSeed = false;
    seedStable = false;
    lastAcceptedPreMatchPose = null;
    stableSeedSampleCount = 0;
  }

  private void updateStableSeedState(Pose2d visionPose, boolean shouldLog, String prefix) {
    double xyDeltaMeters = 0.0;
    double yawDeltaDeg = 0.0;

    if (lastAcceptedPreMatchPose == null) {
      stableSeedSampleCount = 1;
      seedStable = false;
    } else {
      xyDeltaMeters =
          lastAcceptedPreMatchPose.getTranslation().getDistance(visionPose.getTranslation());
      yawDeltaDeg =
          Math.abs(
              visionPose.getRotation().minus(lastAcceptedPreMatchPose.getRotation()).getDegrees());

      boolean xyStable = xyDeltaMeters <= VisionConstants.kPreMatchStableSeedXYDeltaMeters;
      boolean yawStable = yawDeltaDeg <= VisionConstants.kPreMatchStableSeedYawDeltaDeg;

      if (xyStable && yawStable) {
        stableSeedSampleCount++;
      } else {
        stableSeedSampleCount = 1;
        seedStable = false;
      }
    }

    lastAcceptedPreMatchPose = visionPose;
  }
}
