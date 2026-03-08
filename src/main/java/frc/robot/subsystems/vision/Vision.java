package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem using 3× Limelight 4 cameras with a two-phase MegaTag strategy:
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

  private final Drive drive;
  private int configCounter = CONFIG_INTERVAL; // Start at threshold so first loop configures
  private static final int CONFIG_INTERVAL = 250; // Re-send config every 250 loops (~5 seconds)

  // Thermal throttle — skip frames while disabled to keep cameras cool.
  // Per Limelight docs (LDS §13): 100–200 while disabled, 0 while enabled.
  private static final int THROTTLE_DISABLED = 150; // skip 150 frames between processed frames
  private static final int THROTTLE_ENABLED = 0; // process every frame
  private boolean wasDisabled = true; // assume starting disabled

  // Pre-match pose seeding state.
  // The first accepted seed uses setPose() (hard reset); subsequent seeds use
  // addVisionMeasurement() so the estimator converges smoothly.
  private boolean hasInitialSeed = false;

  // Dashboard key for the vision enable/disable toggle.
  // Default: true (vision actively seeds pose estimator).
  // Set to false from Elastic/SmartDashboard to remove vision from the pose
  // estimator for debugging (cameras still run, log data, and seed IMU —
  // only the pose injection is suppressed).
  private static final String kVisionEnabledKey = "Vision/Enabled";

  /**
   * Creates a new Vision subsystem.
   *
   * @param drive The drive subsystem, used to read gyro heading and inject vision measurements.
   */
  public Vision(Drive drive) {
    this.drive = drive;
    // Publish the default value so the toggle appears on the dashboard immediately.
    SmartDashboard.putBoolean(kVisionEnabledKey, true);
  }

  // -----------------------------------------------------------------------
  //  Periodic — runs every 20 ms
  // -----------------------------------------------------------------------
  @Override
  public void periodic() {
    // Read the dashboard toggle — when false, cameras still run and log but
    // do NOT inject measurements into the pose estimator.  Useful for debugging.
    boolean visionEnabled = SmartDashboard.getBoolean(kVisionEnabledKey, true);
    Logger.recordOutput("Vision/enabled", visionEnabled);

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

    // Thermal throttle: skip frames while disabled to keep cameras cool (LDS §13).
    // Only push the throttle value on enable/disable transitions to avoid NT spam.
    boolean isDisabled = DriverStation.isDisabled();
    if (isDisabled != wasDisabled) {
      int throttle = isDisabled ? THROTTLE_DISABLED : THROTTLE_ENABLED;
      for (String name : VisionConstants.kCameraNames) {
        LimelightHelpers.SetThrottle(name, throttle);
      }
      // Reset pre-match seed when transitioning back to disabled (e.g., between
      // practice matches) so the robot re-localizes from scratch.
      if (isDisabled) {
        hasInitialSeed = false;
      }
      wasDisabled = isDisabled;
    }

    // Always feed robot orientation so that IMU seeding (mode 1) works while disabled
    // and MegaTag 2 has up-to-date yaw while enabled.
    // Pass yaw rate so the Limelight can predict orientation between NT updates.
    // Use _NoFlush for all but the last camera to avoid redundant NT flushes.
    double robotYawDeg = drive.getRotation().getDegrees();
    double yawRateDps = Math.toDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond);
    for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
      String name = VisionConstants.kCameraNames[i];
      if (i < VisionConstants.kCameraNames.length - 1) {
        LimelightHelpers.SetRobotOrientation_NoFlush(
            name, robotYawDeg, yawRateDps, 0.0, 0.0, 0.0, 0.0);
      } else {
        // Last camera — flush once to push all orientation updates together.
        LimelightHelpers.SetRobotOrientation(name, robotYawDeg, yawRateDps, 0.0, 0.0, 0.0, 0.0);
      }
    }

    // While disabled, run pre-match pose seeding (strict filters, no pose-jump
    // rejection) so the robot knows its field position before auto starts.
    // While enabled, run normal vision processing with all filters.
    // Both paths still query cameras and log even when visionEnabled is false —
    // only the actual pose injection is suppressed.
    if (isDisabled) {
      for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
        processCameraPreMatch(VisionConstants.kCameraNames[i], i, visionEnabled);
      }
      return;
    }

    // Process each camera (reuse yawRateDps computed above for spin-rejection).
    for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
      processCamera(VisionConstants.kCameraNames[i], robotYawDeg, yawRateDps, i, visionEnabled);
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
    int mode = DriverStation.isDisabled() ? 1 : 4;
    for (String name : VisionConstants.kCameraNames) {
      LimelightHelpers.SetIMUMode(name, mode);
    }
  }

  // -----------------------------------------------------------------------
  //  Per-camera processing
  // -----------------------------------------------------------------------
  /**
   * Queries one Limelight for a MegaTag 2 pose estimate, applies filtering, and — if accepted —
   * injects the measurement into the drive pose estimator.
   *
   * @param cameraName Limelight hostname
   * @param robotYawDeg Current gyro yaw in degrees
   * @param yawRateDegPerSec Current angular velocity in deg/s
   * @param cameraIndex Index into kCameraNames (for logging)
   * @param visionEnabled Whether to inject the measurement into the pose estimator
   */
  private void processCamera(
      String cameraName,
      double robotYawDeg,
      double yawRateDegPerSec,
      int cameraIndex,
      boolean visionEnabled) {

    // 1. Read the MegaTag 2 pose estimate.
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    // 2. Null / no-tag guard.
    if (estimate == null || estimate.tagCount == 0 || estimate.pose == null) {
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      return;
    }

    Pose2d visionPose = estimate.pose;

    // 3. Reject if the robot is spinning too fast (motion blur degrades detection).
    if (Math.abs(yawRateDegPerSec) > VisionConstants.kMaxAngularVelocityDegPerSec) {
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      Logger.recordOutput("Vision/" + cameraName + "/rejectReason", "yaw_rate");
      return;
    }

    // 4. Reject single-tag results that are too small (far away / ambiguous).
    if (estimate.tagCount == 1 && estimate.avgTagArea < VisionConstants.kMinTagAreaForSingleTag) {
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      Logger.recordOutput("Vision/" + cameraName + "/rejectReason", "single_tag_area");
      return;
    }

    // 5. Reject poses that are clearly off the field (with small margin to reject origin).
    if (visionPose.getX() < 0.01
        || visionPose.getX() > Constants.kFieldLengthMeters
        || visionPose.getY() < 0.01
        || visionPose.getY() > Constants.kFieldWidthMeters) {
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      Logger.recordOutput("Vision/" + cameraName + "/rejectReason", "off_field");
      return;
    }

    // 6. Reject large jumps from the current pose estimate (likely a false detection).
    double poseJump = drive.getPose().getTranslation().getDistance(visionPose.getTranslation());
    if (poseJump > VisionConstants.kMaxPoseJumpMeters) {
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      Logger.recordOutput("Vision/" + cameraName + "/rejectReason", "pose_jump");
      return;
    }

    // 7. Read the Limelight's own MegaTag 2 std devs from NetworkTables.
    //    The "stddevs" entry is a 12-element array:
    //      [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1yaw,
    //       MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
    //    We use indices 6 (MT2 X) and 7 (MT2 Y) and take the max as xyStd.
    double[] stddevs = LimelightHelpers.getLimelightNTDoubleArray(cameraName, "stddevs");

    if (stddevs.length < VisionConstants.kExpectedStdDevArrayLength) {
      // Array too short (camera offline or data not yet populated) — skip this cycle.
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      Logger.recordOutput("Vision/" + cameraName + "/rejectReason", "stddevs_missing");
      return;
    }

    double xStdDev = stddevs[VisionConstants.kMT2XStdDevIndex];
    double yStdDev = stddevs[VisionConstants.kMT2YStdDevIndex];
    double xyStdDev = Math.max(xStdDev, yStdDev);

    // If the Limelight reports 0.0 std devs the data isn't valid yet — skip.
    if (xyStdDev <= 0.0) {
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      Logger.recordOutput("Vision/" + cameraName + "/rejectReason", "stddevs_zero");
      return;
    }

    // Apply the MT2 filter-strength multiplier before injecting.
    // >1.0 = trust less (smoother), <1.0 = trust more (snappier).
    double scaledXYStdDev = xyStdDev * VisionConstants.kMT2StdDevMultiplier;

    // 8. Inject into the drive pose estimator (only when vision is enabled on the dashboard).
    if (visionEnabled) {
      drive.addVisionMeasurement(
          visionPose,
          estimate.timestampSeconds,
          VecBuilder.fill(scaledXYStdDev, scaledXYStdDev, VisionConstants.kThetaStdDev));
    }

    // 9. Logging for AdvantageScope (always logged regardless of visionEnabled).
    Logger.recordOutput("Vision/" + cameraName + "/accepted", true);
    Logger.recordOutput("Vision/" + cameraName + "/injected", visionEnabled);
    Logger.recordOutput("Vision/" + cameraName + "/pose", visionPose);
    Logger.recordOutput("Vision/" + cameraName + "/tagCount", estimate.tagCount);
    Logger.recordOutput("Vision/" + cameraName + "/avgTagDist", estimate.avgTagDist);
    Logger.recordOutput("Vision/" + cameraName + "/avgTagArea", estimate.avgTagArea);
    Logger.recordOutput("Vision/" + cameraName + "/xyStdDev", scaledXYStdDev);
    Logger.recordOutput("Vision/" + cameraName + "/rawXYStdDev", xyStdDev);
    Logger.recordOutput("Vision/" + cameraName + "/llXStdDev", xStdDev);
    Logger.recordOutput("Vision/" + cameraName + "/llYStdDev", yStdDev);
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
   */
  private void processCameraPreMatch(String cameraName, int cameraIndex, boolean visionEnabled) {
    // 1. Read the MegaTag 1 pose estimate (full 6-DOF, including yaw).
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);

    // 2. Null / no-tag guard.
    if (estimate == null || estimate.tagCount == 0 || estimate.pose == null) {
      Logger.recordOutput("Vision/" + cameraName + "/preMatch", false);
      return;
    }

    Pose2d visionPose = estimate.pose;

    // 3. Require multi-tag for high confidence (especially important for yaw).
    if (estimate.tagCount < VisionConstants.kPreMatchMinTagCount) {
      Logger.recordOutput("Vision/" + cameraName + "/preMatch", false);
      Logger.recordOutput("Vision/" + cameraName + "/preMatchReject", "tag_count");
      return;
    }

    // 4. Reject poses that are off the field.
    if (visionPose.getX() < 0.01
        || visionPose.getX() > Constants.kFieldLengthMeters
        || visionPose.getY() < 0.01
        || visionPose.getY() > Constants.kFieldWidthMeters) {
      Logger.recordOutput("Vision/" + cameraName + "/preMatch", false);
      Logger.recordOutput("Vision/" + cameraName + "/preMatchReject", "off_field");
      return;
    }

    // 5. Read Limelight MT1 std devs and require high confidence in XY and yaw.
    double[] stddevs = LimelightHelpers.getLimelightNTDoubleArray(cameraName, "stddevs");
    if (stddevs.length < VisionConstants.kExpectedStdDevArrayLength) {
      Logger.recordOutput("Vision/" + cameraName + "/preMatch", false);
      Logger.recordOutput("Vision/" + cameraName + "/preMatchReject", "stddevs_missing");
      return;
    }

    double xStdDev = stddevs[VisionConstants.kMT1XStdDevIndex];
    double yStdDev = stddevs[VisionConstants.kMT1YStdDevIndex];
    double yawStdDev = stddevs[VisionConstants.kMT1YawStdDevIndex];
    double xyStdDev = Math.max(xStdDev, yStdDev);

    if (xyStdDev <= 0.0) {
      Logger.recordOutput("Vision/" + cameraName + "/preMatch", false);
      Logger.recordOutput("Vision/" + cameraName + "/preMatchReject", "stddevs_zero");
      return;
    }

    if (xyStdDev > VisionConstants.kPreMatchMaxStdDev) {
      Logger.recordOutput("Vision/" + cameraName + "/preMatch", false);
      Logger.recordOutput("Vision/" + cameraName + "/preMatchReject", "xy_stddev_too_high");
      return;
    }

    if (yawStdDev > VisionConstants.kPreMatchMaxYawStdDevDeg) {
      Logger.recordOutput("Vision/" + cameraName + "/preMatch", false);
      Logger.recordOutput("Vision/" + cameraName + "/preMatchReject", "yaw_stddev_too_high");
      return;
    }

    // 6. Seed the pose estimator.
    //    First accepted result → hard reset (setPose) with the FULL MT1 pose
    //    including yaw.  This resets the gyro offset in the pose estimator so
    //    the Pigeon2 heading now maps to the MT1-detected field heading.
    //    Subsequent results → soft update to refine position.
    //    When visionEnabled is false, we still mark hasInitialSeed so that
    //    re-enabling doesn't trigger an unexpected hard reset, but we skip
    //    the actual pose estimator injection.
    if (!hasInitialSeed) {
      if (visionEnabled) {
        drive.setPose(visionPose); // Full pose including MT1 yaw
      }
      hasInitialSeed = true;
      Logger.recordOutput(
          "Vision/" + cameraName + "/preMatchAction",
          visionEnabled ? "setPose" : "setPose_suppressed");
      Logger.recordOutput(
          "Vision/" + cameraName + "/preMatchSeededYawDeg", visionPose.getRotation().getDegrees());
    } else {
      // After the initial seed, keep refining position.  We trust MT1 yaw
      // with moderate weight (not 999999) since the gyro offset is already set.
      // Apply the MT1 filter-strength multiplier to both XY and yaw stddevs.
      double scaledXYStdDev = xyStdDev * VisionConstants.kMT1StdDevMultiplier;
      double scaledYawStdDev = yawStdDev * VisionConstants.kMT1StdDevMultiplier;
      if (visionEnabled) {
        drive.addVisionMeasurement(
            visionPose,
            estimate.timestampSeconds,
            VecBuilder.fill(scaledXYStdDev, scaledXYStdDev, Math.toRadians(scaledYawStdDev)));
      }
      Logger.recordOutput(
          "Vision/" + cameraName + "/preMatchAction",
          visionEnabled ? "refine" : "refine_suppressed");
    }

    // 7. Logging.
    Logger.recordOutput("Vision/" + cameraName + "/preMatch", true);
    Logger.recordOutput("Vision/" + cameraName + "/preMatchInjected", visionEnabled);
    Logger.recordOutput("Vision/" + cameraName + "/preMatchPose", visionPose);
    Logger.recordOutput("Vision/" + cameraName + "/preMatchTagCount", estimate.tagCount);
    Logger.recordOutput("Vision/" + cameraName + "/preMatchXYStdDev", xyStdDev);
    Logger.recordOutput("Vision/" + cameraName + "/preMatchYawStdDev", yawStdDev);
  }
}
