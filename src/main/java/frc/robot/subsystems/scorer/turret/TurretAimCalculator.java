package frc.robot.subsystems.scorer.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldConstants;
import frc.robot.Constants;

/**
 * Pure-math utility that computes the desired turret (lateral) and hood (vertical) angles for the
 * scorers based on the robot's current field pose and alliance color.
 *
 * <p>Both turrets are mechanically parallel, so a single aim solution is computed from the midpoint
 * between the two shooter exits and the same angles are returned for left and right.
 *
 * <h2>Zone logic</h2>
 *
 * <ul>
 *   <li><b>Own-alliance zone</b> — aim both scorers at the alliance hub.
 *   <li><b>Neutral (middle) zone</b> — "pass" mode: aim at the nearest landing zone on the alliance
 *       side so a partner can receive the ball.
 *   <li><b>Opponent zone</b> — stow (return 0° turret, stow hood).
 * </ul>
 *
 * <h2>Coordinate conventions</h2>
 *
 * <ul>
 *   <li>Field origin at blue-alliance corner (0, 0). X runs toward red wall.
 *   <li>Robot heading 0° = facing red wall (+X).
 *   <li>Turret 0° = facing robot-forward. Positive = CCW (left when viewed from above).
 *   <li>Hood angle = elevation above horizontal (10°–35° limits from ScorerConstants).
 * </ul>
 */
public final class TurretAimCalculator {

  /** Immutable result of a single aim calculation for both scorers. */
  public static class AimResult {
    /** Left turret lateral angle in degrees (0 = forward, + = left). */
    public final double leftTurretDeg;
    /** Left hood vertical angle in degrees (elevation above horizontal). */
    public final double leftHoodDeg;
    /** Right turret lateral angle in degrees (0 = forward, + = left). */
    public final double rightTurretDeg;
    /** Right hood vertical angle in degrees (elevation above horizontal). */
    public final double rightHoodDeg;

    /** The field-space target both scorers are aiming at (for logging). */
    public final Translation2d target;

    public AimResult(
        double leftTurretDeg,
        double leftHoodDeg,
        double rightTurretDeg,
        double rightHoodDeg,
        Translation2d target) {
      this.leftTurretDeg = leftTurretDeg;
      this.leftHoodDeg = leftHoodDeg;
      this.rightTurretDeg = rightTurretDeg;
      this.rightHoodDeg = rightHoodDeg;
      this.target = target;
    }
  }

  // Hood limits from ScorerConstants (duplicated here to avoid circular dep)
  private static final double HOOD_MIN_DEG = 10.0;
  private static final double HOOD_MAX_DEG = 35.0;

  /** Prevent instantiation. */
  private TurretAimCalculator() {}

  /**
   * Compute aim angles for both turrets and hoods.
   *
   * @param robotPose Current robot field pose from odometry / pose estimator.
   * @param allianceColor 'R' for red, 'B' for blue.
   * @return An {@link AimResult} with four angles plus debug info.
   */
  public static AimResult calculate(Pose2d robotPose) {
    boolean isBlue = (Constants.alliance == Alliance.Blue);

    FieldConstants.Zone zone = Constants.currentZone;
    // ---- Pick field-space target ----
    Translation2d fieldTarget;
    double targetZ; // height of the target above the floor (meters)

    boolean home = false;
    if (isBlue && zone == FieldConstants.Zone.BLUE) {
      home = true;
    } else if (!isBlue && zone == FieldConstants.Zone.RED) {
      home = true;
    }

    if (home) {
      // Aim at our hub
      if (isBlue) {
        fieldTarget =
            new Translation2d(FieldConstants.Hub.BLUE.getX(), FieldConstants.Hub.BLUE.getY());
        targetZ = FieldConstants.Hub.BLUE.getZ();
      } else {
        fieldTarget =
            new Translation2d(FieldConstants.Hub.RED.getX(), FieldConstants.Hub.RED.getY());
        targetZ = FieldConstants.Hub.RED.getZ();
      }
    } else { // Midfield or Opponents

      // Pass mode — aim at the landing zone on OUR side.
      // Choose whichever landing zone (outpost vs depot) is closer to the
      // robot's current Y to minimize turret travel.
      fieldTarget = pickLandingTarget(robotPose.getY(), isBlue);
      // Landing zones are on the floor — target Z = 0 (lob arc, not a line
      // drive). We'll set a fixed hood angle for passing.
      targetZ = 0.0;
    }

    // ---- Compute aim from midpoint of the two shooter exits ----
    // Both turrets are parallel, so we use a single aim solution from the
    // midpoint between left and right shooter positions.
    Rotation2d heading = robotPose.getRotation();

    double midShooterX =
        (Constants.ScorerConstants.kLeftShooterXOffsetMeters
                + Constants.ScorerConstants.kRightShooterXOffsetMeters)
            / 2.0;
    double midShooterY =
        (Constants.ScorerConstants.kLeftShooterYOffsetMeters
                + Constants.ScorerConstants.kRightShooterYOffsetMeters)
            / 2.0;

    Translation2d midShooterField = robotToField(robotPose, midShooterX, midShooterY);
    double[] angles = computeAngles(midShooterField, fieldTarget, targetZ, heading);

    double turretDeg = angles[0];
    double hoodDeg = angles[1];

    // Same angles for both sides (parallel turrets)
    return new AimResult(turretDeg, hoodDeg, turretDeg, hoodDeg, fieldTarget);
  }

  // ====================================================================
  // Private helpers
  // ====================================================================

  /**
   * Convert a robot-relative offset to a field-space Translation2d using the robot's current pose.
   */
  private static Translation2d robotToField(Pose2d robotPose, double robotRelX, double robotRelY) {
    // Rotate the offset by the robot heading, then translate to the robot's
    // field position.
    double cos = robotPose.getRotation().getCos();
    double sin = robotPose.getRotation().getSin();
    double fieldX = robotPose.getX() + (robotRelX * cos - robotRelY * sin);
    double fieldY = robotPose.getY() + (robotRelX * sin + robotRelY * cos);
    return new Translation2d(fieldX, fieldY);
  }

  /**
   * Compute turret and hood angles for a single shooter.
   *
   * @param shooterField Field-space XY of the shooter exit.
   * @param targetField Field-space XY of the target.
   * @param targetZ Target height above floor (m).
   * @param robotHeading Current robot heading on the field.
   * @return double[2]: [turretDeg, hoodDeg].
   */
  private static double[] computeAngles(
      Translation2d shooterField,
      Translation2d targetField,
      double targetZ,
      Rotation2d robotHeading) {
    // --- Lateral (turret) angle ---
    // Vector from shooter to target in field frame
    double dx = targetField.getX() - shooterField.getX();
    double dy = targetField.getY() - shooterField.getY();

    // Field-frame bearing to the target (radians, 0 = +X, CCW positive)
    double fieldBearing = Math.atan2(dy, dx);

    // Turret angle = field bearing − robot heading.
    // This gives the angle in the robot's reference frame.
    double turretRad = fieldBearing - robotHeading.getRadians();

    // Normalize to [−π, π]
    turretRad = Math.atan2(Math.sin(turretRad), Math.cos(turretRad));
    double turretDeg = Math.toDegrees(turretRad);

    // Clamp turret to physical limits
    if (turretDeg > Constants.ScorerConstants.kTurretMaxPositionUnits) {
      turretDeg = 360.0 - turretDeg; // wrap around to negative angles
    } else if (turretDeg < Constants.ScorerConstants.kTurretMinPositionUnits) {
      turretDeg = 360.0 + turretDeg; // wrap around to positive angles
    }

    // --- Vertical (hood) angle ---
    double horizontalDist = Math.hypot(dx, dy);
    double dz = targetZ - Constants.ScorerConstants.kShooterExitZMeters;
    double hoodDeg =
        Math.toDegrees(
            Math.atan2(
                dz,
                horizontalDist)); // TODO tune this formula for a better arc (currently just a line
    // drive)

    // Clamp hood to physical limits
    hoodDeg = clamp(hoodDeg, HOOD_MIN_DEG, HOOD_MAX_DEG);

    return new double[] {turretDeg, hoodDeg};
  }

  /**
   * Pick the closer landing zone for pass mode.
   *
   * @param robotY Robot's current Y coordinate on the field (meters).
   * @param isBlue true if we are the blue alliance.
   * @return Field Translation2d of the chosen landing zone center.
   */
  private static Translation2d pickLandingTarget(double robotY, boolean isBlue) {
    double outpostY, depotY, targetX;
    if (isBlue) {
      outpostY = FieldConstants.Corner.BLUEOUT.getY();
      depotY = FieldConstants.Corner.BLUEDEP.getY();
      targetX = FieldConstants.Corner.BLUEOUT.getX(); // same X for both blue landing zones
    } else {
      outpostY = FieldConstants.Corner.REDOUT.getY();
      depotY = FieldConstants.Corner.REDDEP.getY();
      targetX = FieldConstants.Corner.REDOUT.getX(); // same X for both red landing zones
    }

    // Pick whichever landing zone is closer to the robot in Y
    double distToOutpost = Math.abs(robotY - outpostY);
    double distToDepot = Math.abs(robotY - depotY);
    double chosenY = (distToOutpost <= distToDepot) ? outpostY : depotY;

    return new Translation2d(targetX, chosenY);
  }

  private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
  }
}
