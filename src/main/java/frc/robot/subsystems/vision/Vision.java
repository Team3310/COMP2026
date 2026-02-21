package frc.robot.subsystems.vision;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.LimelightHelpers.PoseEstimate;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.drive.Drive;
import org.littletonrobotics.junction.Logger;

/**
 * Vision subsystem using MegaTag 2 pose estimation with 3× Limelight 4 cameras. Each loop we:
 *
 * <ol>
 *   <li>Feed the robot's current gyro yaw to every Limelight via {@code SetRobotOrientation()}.
 *   <li>Query {@code getBotPoseEstimate_wpiBlue_MegaTag2()} for each camera.
 *   <li>Filter out bad results (no tags, spinning too fast, off-field, big jumps).
 *   <li>Read the Limelight's own MegaTag 2 standard deviations from NetworkTables (the {@code
 *       stddevs} entry) and use {@code max(xStd, yStd)} as the XY trust weight.
 *   <li>Feed accepted measurements into {@code Drive.addVisionMeasurement()}.
 * </ol>
 *
 * <p>Theta (rotation) std dev is set to 999999 because MegaTag 2 does <b>not</b> estimate rotation
 * — it uses the gyro heading you supply.
 *
 * <p>Std dev approach adapted from Team 254's 2025 VisionSubsystem: rather than computing our own
 * heuristic, we trust the uncertainty values that the Limelight solver itself produces.
 */
public class Vision extends SubsystemBase {

  private final Drive drive;
  private int configCounter = CONFIG_INTERVAL; // Start at threshold so first loop configures
  private static final int CONFIG_INTERVAL = 250; // Re-send config every 250 loops (~5 seconds)

  /**
   * Creates a new Vision subsystem.
   *
   * @param drive The drive subsystem, used to read gyro heading and inject vision measurements.
   */
  public Vision(Drive drive) {
    this.drive = drive;
  }

  // -----------------------------------------------------------------------
  //  Periodic — runs every 20 ms
  // -----------------------------------------------------------------------
  @Override
  public void periodic() {
    // Periodically re-send camera config so cameras that boot late or power-cycle
    // mid-match still get the correct poses and IMU mode.
    configCounter++;
    if (configCounter >= CONFIG_INTERVAL) {
      configureCameras();
      configCounter = 0;
    }

    // Don't process vision while disabled — no point in seeding when we might be moving the robot
    // manually, and it avoids spamming pose estimator with stale data.
    if (DriverStation.isDisabled()) {
      return;
    }

    // Current gyro heading — MegaTag 2 needs this to solve for XY-only pose.
    double robotYawDeg = drive.getRotation().getDegrees();

    // Current angular velocity — used to reject updates during fast spins.
    double yawRateDegPerSec = Math.toDegrees(drive.getChassisSpeeds().omegaRadiansPerSecond);

    // Process each camera
    for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
      processCamera(VisionConstants.kCameraNames[i], robotYawDeg, yawRateDegPerSec, i);
    }
  }

  // -----------------------------------------------------------------------
  //  Camera configuration (called once)
  // -----------------------------------------------------------------------
  /**
   * Pushes the measured camera poses into each Limelight and sets the IMU mode to "external
   * orientation" (mode 0), which tells MegaTag 2 to use the yaw we feed via SetRobotOrientation().
   */
  private void configureCameras() {
    for (int i = 0; i < VisionConstants.kCameraNames.length; i++) {
      String name = VisionConstants.kCameraNames[i];
      double[] pose = VisionConstants.kCameraPoses[i];

      // Set camera position relative to robot center.
      // Args: forward, side, up, roll, pitch, yaw (meters / degrees)
      LimelightHelpers.setCameraPose_RobotSpace(
          name, pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);

      // IMU mode 0 = use external orientation data (our gyro).
      // This is required for MegaTag 2 to function correctly.
      LimelightHelpers.SetIMUMode(name, 0);
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
   */
  private void processCamera(
      String cameraName, double robotYawDeg, double yawRateDegPerSec, int cameraIndex) {

    // 1. Feed robot orientation to this Limelight so MegaTag 2 can solve XY.
    LimelightHelpers.SetRobotOrientation(cameraName, robotYawDeg, 0.0, 0.0, 0.0, 0.0, 0.0);

    // 2. Read the MegaTag 2 pose estimate.
    PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

    // 3. Null / no-tag guard.
    if (estimate == null || estimate.tagCount == 0 || estimate.pose == null) {
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      return;
    }

    Pose2d visionPose = estimate.pose;

    // 4. Reject if the robot is spinning too fast (motion blur degrades detection).
    if (Math.abs(yawRateDegPerSec) > VisionConstants.kMaxAngularVelocityDegPerSec) {
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      Logger.recordOutput("Vision/" + cameraName + "/rejectReason", "yaw_rate");
      return;
    }

    // 5. Reject single-tag results that are too small (far away / ambiguous).
    if (estimate.tagCount == 1 && estimate.avgTagArea < VisionConstants.kMinTagAreaForSingleTag) {
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      Logger.recordOutput("Vision/" + cameraName + "/rejectReason", "single_tag_area");
      return;
    }

    // 6. Reject poses that are clearly off the field (with small margin to reject origin).
    if (visionPose.getX() < 0.01
        || visionPose.getX() > Constants.kFieldLengthMeters
        || visionPose.getY() < 0.01
        || visionPose.getY() > Constants.kFieldWidthMeters) {
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      Logger.recordOutput("Vision/" + cameraName + "/rejectReason", "off_field");
      return;
    }

    // 7. Reject large jumps from the current pose estimate (likely a false detection).
    double poseJump = drive.getPose().getTranslation().getDistance(visionPose.getTranslation());
    if (poseJump > VisionConstants.kMaxPoseJumpMeters) {
      Logger.recordOutput("Vision/" + cameraName + "/accepted", false);
      Logger.recordOutput("Vision/" + cameraName + "/rejectReason", "pose_jump");
      return;
    }

    // 8. Read the Limelight's own MegaTag 2 std devs from NetworkTables.
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

    // 9. Inject into the drive pose estimator.
    drive.addVisionMeasurement(
        visionPose,
        estimate.timestampSeconds,
        VecBuilder.fill(xyStdDev, xyStdDev, VisionConstants.kThetaStdDev));

    // 10. Logging for AdvantageScope.
    Logger.recordOutput("Vision/" + cameraName + "/accepted", true);
    Logger.recordOutput("Vision/" + cameraName + "/pose", visionPose);
    Logger.recordOutput("Vision/" + cameraName + "/tagCount", estimate.tagCount);
    Logger.recordOutput("Vision/" + cameraName + "/avgTagDist", estimate.avgTagDist);
    Logger.recordOutput("Vision/" + cameraName + "/avgTagArea", estimate.avgTagArea);
    Logger.recordOutput("Vision/" + cameraName + "/xyStdDev", xyStdDev);
    Logger.recordOutput("Vision/" + cameraName + "/llXStdDev", xStdDev);
    Logger.recordOutput("Vision/" + cameraName + "/llYStdDev", yStdDev);
  }
}
