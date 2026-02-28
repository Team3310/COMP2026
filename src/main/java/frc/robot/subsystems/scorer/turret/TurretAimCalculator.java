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
 *   <li>Hood angle = degrees from vertical (0° = straight up, 35° = 55° from horizontal). The ball
 *       exits perpendicular to the hood face, so the actual launch elevation from horizontal is
 *       {@code 90 − hoodDeg}. Lower hood values produce steeper (more vertical) trajectories;
 *       higher values produce flatter trajectories.
 * </ul>
 */
public final class TurretAimCalculator {

  /** Immutable result of a single aim calculation for both scorers. */
  public static class AimResult {
    /** Left turret lateral angle in degrees (0 = forward, + = left). */
    public final double leftTurretDeg;
    /** Left hood angle in degrees from vertical (0 = straight up, 35 = 55° from horizontal). */
    public final double leftHoodDeg;
    /** Left vertical-feeder speed in RPM (0 = off). */
    public final double leftFeederRPM;
    /** Right turret lateral angle in degrees (0 = forward, + = left). */
    public final double rightTurretDeg;
    /** Right hood angle in degrees from vertical (0 = straight up, 35 = 55° from horizontal). */
    public final double rightHoodDeg;
    /** Right vertical-feeder speed in RPM (0 = off). */
    public final double rightFeederRPM;

    /** The field-space target both scorers are aiming at (for logging). */
    public final Translation2d target;

    /** True when in own-alliance zone (aim at hub); false for pass/lob/stow. */
    public final boolean home;

    public AimResult(
        double leftTurretDeg,
        double leftHoodDeg,
        double leftFeederRPM,
        double rightTurretDeg,
        double rightHoodDeg,
        double rightFeederRPM,
        Translation2d target,
        boolean home) {
      this.leftTurretDeg = leftTurretDeg;
      this.leftHoodDeg = leftHoodDeg;
      this.leftFeederRPM = leftFeederRPM;
      this.rightTurretDeg = rightTurretDeg;
      this.rightHoodDeg = rightHoodDeg;
      this.rightFeederRPM = rightFeederRPM;
      this.target = target;
      this.home = home;
    }
  }

  // Hood limits — read from Constants so they stay tunable in one place.
  // (No circular dependency: Constants is a leaf class with only static finals.)

  // ---- Turret wrap-around hysteresis state ----
  // Remembers the last turret output so that the normalization picks the
  // closest equivalent angle each cycle.  The turret only "flips" when the
  // target genuinely crosses past ±220° — not because of a stateless modulo
  // that doesn't know which side we're on.
  private static double lastTurretDeg = 0.0;

  /** Prevent instantiation. */
  private TurretAimCalculator() {}

  /**
   * Compute aim angles for both turrets and hoods.
   *
   * @param robotPose Current robot field pose from odometry / pose estimator.
   * @return An {@link AimResult} with six values (L/R turret, hood, feeder) plus debug info.
   */
  public static AimResult calculate(Pose2d robotPose) {
    boolean isBlue = (Constants.alliance == Alliance.Blue);

    FieldConstants.Zone zone = Constants.currentZone;

    // ---- Trench zone: stow hood to lowest angle, zero feeder ----
    boolean inTrench =
        (zone == FieldConstants.Zone.BLUETRENCH || zone == FieldConstants.Zone.REDTRENCH);

    // ---- Pick field-space target ----
    Translation2d fieldTarget;

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
      } else {
        fieldTarget =
            new Translation2d(FieldConstants.Hub.RED.getX(), FieldConstants.Hub.RED.getY());
      }
    } else { // Midfield, trench, or opponent zone
      // Pass mode — aim at the landing zone on OUR side.
      // Choose whichever landing zone (outpost vs depot) is closer to the
      // robot's current Y to minimize turret travel.
      fieldTarget = pickLandingTarget(robotPose.getY(), isBlue);
    }

    // ---- Compute aim from midpoint of the two shooter exits ----
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
    double[] result = computeAngles(midShooterField, fieldTarget, heading, home, inTrench);

    double turretDeg = result[0];
    double hoodDeg = result[1];
    double feederRPM = result[2];

    // Same values for both sides (parallel turrets)
    return new AimResult(
        turretDeg, hoodDeg, feederRPM, turretDeg, hoodDeg, feederRPM, fieldTarget, home);
  }

  // ====================================================================
  // Private helpers
  // ====================================================================

  /**
   * Estimate the ball's time-of-flight in seconds for a hub shot at the given distance.
   *
   * <p>Interpolates from the TOF column (column 3) of {@link Constants.ScorerConstants#kHubTable}.
   * Returns 0 if not in hub-scoring mode (pass/lob shots don't use aim-ahead lead).
   *
   * @param distance Horizontal distance in meters from shooter to target.
   * @param home true when in own-alliance zone (hub scoring); false for pass mode.
   * @return Estimated flight time in seconds, or 0 if not in hub mode.
   */
  public static double estimateTimeOfFlight(double distance, boolean home) {
    if (!home) {
      return 0.0; // No lead for pass/lob shots
    }
    return interpolateTable(Constants.ScorerConstants.kHubTable, distance, 3);
  }

  /**
   * Convert a robot-relative offset to a field-space Translation2d using the robot's current pose.
   */
  public static Translation2d robotToField(Pose2d robotPose, double robotRelX, double robotRelY) {
    // Rotate the offset by the robot heading, then translate to the robot's
    // field position.
    double cos = robotPose.getRotation().getCos();
    double sin = robotPose.getRotation().getSin();
    double fieldX = robotPose.getX() + (robotRelX * cos - robotRelY * sin);
    double fieldY = robotPose.getY() + (robotRelX * sin + robotRelY * cos);
    return new Translation2d(fieldX, fieldY);
  }

  /**
   * Compute turret angle, hood angle, and feeder speed for a single shooter.
   *
   * @param shooterField Field-space XY of the shooter exit.
   * @param targetField Field-space XY of the target.
   * @param robotHeading Current robot heading on the field.
   * @param home true when in own alliance zone (aim at hub); false for pass/lob mode.
   * @param inTrench true when in a trench zone — forces hood to minimum and feeder off.
   * @return double[3]: [turretDeg, hoodDeg, feederRPM].
   */
  private static double[] computeAngles(
      Translation2d shooterField,
      Translation2d targetField,
      Rotation2d robotHeading,
      boolean home,
      boolean inTrench) {
    // --- Lateral (turret) angle ---
    // Vector from shooter to target in field frame
    double dx = targetField.getX() - shooterField.getX();
    double dy = targetField.getY() - shooterField.getY();

    // Field-frame bearing to the target (radians, 0 = +X, CCW positive)
    double fieldBearing = Math.atan2(dy, dx);

    // Turret angle = field bearing − robot heading.
    // This gives the angle in the robot's reference frame.
    double turretRad = fieldBearing - robotHeading.getRadians();

    // ---- Wrap-around with hysteresis ----
    // The turret can physically travel ±220° from forward (440° total range).
    // The 40° overlap (±180° to ±220°) on each side gives us hysteresis:
    //   - We prefer to stay on the current wrap (closest to lastTurretDeg)
    //   - If that wrap exceeds ±220°, we try the OTHER wrap (±360°)
    //   - If the other wrap is in range, we flip (go the long way around)
    //   - If neither wrap is in range (dead-band behind robot), clamp to nearest limit
    double turretDeg = Math.toDegrees(turretRad);
    double turretMax = Constants.ScorerConstants.kTurretMaxPositionUnits; // +220
    double turretMin = Constants.ScorerConstants.kTurretMinPositionUnits; // -220

    // Step 1: Pick the wrap closest to where we were last cycle.
    double delta = turretDeg - lastTurretDeg;
    delta = delta % 360.0;
    if (delta > 180.0) {
      delta -= 360.0;
    } else if (delta <= -180.0) {
      delta += 360.0;
    }
    double preferred = lastTurretDeg + delta;

    // Step 2: If preferred wrap is in range, use it.  Otherwise try the
    // other wrap (±360°).  If that's in range, flip to it.  If neither is
    // in range the target is in the ~280° dead-band behind the hard-stops,
    // so clamp to whichever limit is closer.
    if (preferred >= turretMin && preferred <= turretMax) {
      turretDeg = preferred;
    } else {
      // Try the other 360° wrap
      double alternate = (preferred > turretMax) ? preferred - 360.0 : preferred + 360.0;
      if (alternate >= turretMin && alternate <= turretMax) {
        // Other wrap is reachable — flip the long way around
        turretDeg = alternate;
      } else {
        // Neither wrap fits — clamp to the closer limit
        double distPrefMin = Math.abs(preferred - turretMin);
        double distPrefMax = Math.abs(preferred - turretMax);
        double distAltMin = Math.abs(alternate - turretMin);
        double distAltMax = Math.abs(alternate - turretMax);
        double minDist =
            Math.min(Math.min(distPrefMin, distPrefMax), Math.min(distAltMin, distAltMax));
        if (minDist == distPrefMin || minDist == distAltMin) {
          turretDeg = turretMin;
        } else {
          turretDeg = turretMax;
        }
      }
    }

    // Remember this output for next cycle's hysteresis
    lastTurretDeg = turretDeg;

    // --- Vertical (hood) angle and feeder speed via lookup table ---
    double horizontalDist = Math.hypot(dx, dy);
    double hoodDeg;
    double feederRPM;

    if (inTrench) {
      // Trench zone: hood as low (steep) as possible, feeder off.
      hoodDeg = Constants.ScorerConstants.kHoodMinDegrees;
      feederRPM = Constants.ScorerConstants.kFeederStowRPM;
    } else {
      // Pick the correct lookup table
      double[][] table =
          home ? Constants.ScorerConstants.kHubTable : Constants.ScorerConstants.kPassTable;

      hoodDeg = interpolateTable(table, horizontalDist, 1); // column 1 = hood
      feederRPM = interpolateTable(table, horizontalDist, 2); // column 2 = feeder

      // Clamp hood to physical limits
      hoodDeg =
          clamp(
              hoodDeg,
              Constants.ScorerConstants.kHoodMinDegrees,
              Constants.ScorerConstants.kHoodMaxDegrees);
    }

    return new double[] {turretDeg, hoodDeg, feederRPM};
  }

  /**
   * Linearly interpolate a value from a sorted lookup table.
   *
   * <p>The table must be sorted ascending by column 0 (distance). If the input distance is below
   * the first row or above the last row, the corresponding row's value is returned (clamped, not
   * extrapolated).
   *
   * @param table 2-D array where each row is {distance, ...values...}.
   * @param distance The horizontal distance to look up.
   * @param valueColumn The column index of the value to interpolate (1-based: 1 = hood, 2 = feeder,
   *     3 = TOF).
   * @return The interpolated value.
   */
  public static double interpolateTable(double[][] table, double distance, int valueColumn) {
    // Below first row — clamp
    if (distance <= table[0][0]) {
      return table[0][valueColumn];
    }
    // Above last row — clamp
    if (distance >= table[table.length - 1][0]) {
      return table[table.length - 1][valueColumn];
    }
    // Find the bracketing rows and lerp
    for (int i = 0; i < table.length - 1; i++) {
      double d0 = table[i][0];
      double d1 = table[i + 1][0];
      if (distance >= d0 && distance <= d1) {
        double t = (distance - d0) / (d1 - d0);
        return table[i][valueColumn] + t * (table[i + 1][valueColumn] - table[i][valueColumn]);
      }
    }
    // Fallback (should never reach here if table is sorted)
    return table[table.length - 1][valueColumn];
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
