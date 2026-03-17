package frc.robot.subsystems.scorer.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ScorerConstants;
import frc.robot.Constants.SimPhysicsConstants;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

/**
 * Subsystem that runs the {@link TurretAimCalculator} every cycle, writes the results into an
 * AdvantageKit IO layer, and logs everything so you can inspect the four aim values in
 * AdvantageScope during simulation (or replay).
 *
 * <p>This class does <b>not</b> command the physical turret/hood motors — it only computes and logs
 * the desired angles. Wire those angles into the actual {@link Turret} and {@link
 * frc.robot.subsystems.scorer.hood.Hood} subsystems when you're ready.
 */
public class TurretAimManager extends SubsystemBase {
  private static final int LOG_INTERVAL = 5; // ~10 Hz at a 20 ms main loop

  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> speedsSupplier;
  private final TurretAimIOInputsAutoLogged inputs = new TurretAimIOInputsAutoLogged();

  // Cached latest result for external consumers
  private TurretAimCalculator.AimResult latestResult = null;

  // ---- Turret command deadband state ----
  // Holds the last output angle so small corrections below kTurretDeadbandDeg
  // are suppressed, keeping the turret steady while shooting.
  private double prevLeftTurretDeg = 0.0;
  private double prevRightTurretDeg = 0.0;

  // ---- Simulated turret inertia state ----
  // Tracks where the turret *physically is* (with lag), vs. where the
  // calculator *wants* it to be (instant).  Used purely for AdvantageScope
  // visualisation — the logged ghost Pose2d rotates with realistic inertia.
  private double simTurretAngleDeg = 0.0; // current simulated turret angle
  private double prevTimestamp = -1.0; // for dt calculation
  private int logCounter = 0;

  /**
   * @param poseSupplier Supplies the robot's current field pose (usually {@code drive::getPose}).
   * @param speedsSupplier Supplies the robot's current robot-relative chassis speeds (usually
   *     {@code drive::getChassisSpeeds}). Used for aim-ahead prediction.
   */
  public TurretAimManager(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
  }

  @Override
  public void periodic() {
    logCounter++;
    boolean shouldLog = logCounter >= LOG_INTERVAL;
    if (shouldLog) {
      logCounter = 0;
    }

    Pose2d pose = poseSupplier.get();
    char alliance = getAllianceChar();
    ChassisSpeeds robotSpeeds = speedsSupplier.get();

    // ================================================================
    // Step 1: Phase-delay compensation
    // ================================================================
    // Shift the estimated pose forward by kPhaseDelaySeconds to account for
    // sensor / processing pipeline latency.  Pose2d.exp(Twist2d) properly
    // handles the arc the robot follows during rotation (unlike linear extrap).
    double phaseDelay = ScorerConstants.kPhaseDelaySeconds;
    Pose2d phaseCorrectedPose =
        pose.exp(
            new Twist2d(
                robotSpeeds.vxMetersPerSecond * phaseDelay,
                robotSpeeds.vyMetersPerSecond * phaseDelay,
                robotSpeeds.omegaRadiansPerSecond * phaseDelay));

    // ================================================================
    // Step 2: Predict the pose at ball release time
    // ================================================================
    // Phase delay compensates sensing/estimation latency. Release delay covers
    // the time between deciding to fire and the ball actually leaving the robot.
    double releaseDelay = ScorerConstants.kReleaseDelaySeconds;
    Pose2d releasePose =
        phaseCorrectedPose.exp(
            new Twist2d(
                robotSpeeds.vxMetersPerSecond * releaseDelay,
                robotSpeeds.vyMetersPerSecond * releaseDelay,
                robotSpeeds.omegaRadiansPerSecond * releaseDelay));

    // We need this first pass to determine (a) the target location and (b)
    // whether we're in hub mode (home=true) so we know if TOF lead applies.
    TurretAimCalculator.AimResult initialResult = TurretAimCalculator.calculate(releasePose);

    // ================================================================
    // Step 3: Iterative TOF-based aim-ahead convergence
    // ================================================================
    // The time-of-flight depends on distance, which changes with the velocity
    // offset, which depends on TOF — a circular dependency.  We iterate:
    //   1. Estimate TOF from current shooter→target distance
    //   2. Extrapolate shooter position by fieldVelocity × TOF
    //   3. Recompute distance from extrapolated position to target
    //   4. Repeat until converged
    //
    // Only hub shots use TOF lead.  Pass/lob shots aim at a large landing zone,
    // so lead is unnecessary (TOF returns 0 for non-home zones).

    // Compute field-relative velocity at the release heading.
    double cosH = releasePose.getRotation().getCos();
    double sinH = releasePose.getRotation().getSin();
    double fieldVx = robotSpeeds.vxMetersPerSecond * cosH - robotSpeeds.vyMetersPerSecond * sinH;
    double fieldVy = robotSpeeds.vxMetersPerSecond * sinH + robotSpeeds.vyMetersPerSecond * cosH;

    // Midpoint of the two shooter exits (robot-relative)
    double midShooterX =
        (Constants.ScorerConstants.kLeftShooterXOffsetMeters
                + Constants.ScorerConstants.kRightShooterXOffsetMeters)
            / 2.0;
    double midShooterY =
        (Constants.ScorerConstants.kLeftShooterYOffsetMeters
                + Constants.ScorerConstants.kRightShooterYOffsetMeters)
            / 2.0;

    // Shooter position in field space at release.
    Translation2d shooterField =
        TurretAimCalculator.robotToField(releasePose, midShooterX, midShooterY);
    Translation2d target = initialResult.target;
    boolean home = initialResult.home;

    // Iterative convergence loop
    double convergedTof = 0.0;
    Translation2d lookaheadShooter = shooterField;
    double lookaheadDist = shooterField.getDistance(target);

    for (int i = 0; i < ScorerConstants.kTofIterations; i++) {
      convergedTof = TurretAimCalculator.estimateTimeOfFlight(lookaheadDist, home);
      if (convergedTof <= 0.0) break; // No lead (pass mode or zero TOF)

      // Extrapolate shooter position by field velocity × TOF
      lookaheadShooter =
          new Translation2d(
              shooterField.getX() + fieldVx * convergedTof,
              shooterField.getY() + fieldVy * convergedTof);
      lookaheadDist = lookaheadShooter.getDistance(target);
    }

    // ================================================================
    // Step 4: Final ballistic compensation
    // ================================================================
    // Translate the release pose by field velocity × TOF so the aim accounts
    // for the robot's translational carry during flight. Keep the release
    // heading — continued robot rotation after launch does not rotate the ball.
    Pose2d ballisticPose;
    if (convergedTof > 0.0) {
      ballisticPose =
          new Pose2d(
              releasePose.getX() + fieldVx * convergedTof,
              releasePose.getY() + fieldVy * convergedTof,
              releasePose.getRotation());
    } else {
      ballisticPose = releasePose;
    }

    // Run the aim calculator on the ballistically compensated pose.
    TurretAimCalculator.AimResult result = TurretAimCalculator.calculate(ballisticPose);
    latestResult = result;

    // ---- Simulated turret inertia ----
    // The turret is physically limited to [kTurretMinPositionUnits, kTurretMaxPositionUnits].
    // It CANNOT wrap around through the dead zone behind the robot — when the
    // commanded angle flips (e.g. +219° → -219°) the turret must slew the long
    // way through 0°.  We model this as simple first-order lag on the raw angle
    // with a velocity clamp — no shortest-path wrapping.
    double now = Timer.getFPGATimestamp();
    double dt = (prevTimestamp < 0) ? 0.02 : (now - prevTimestamp);
    prevTimestamp = now;

    double targetAngleDeg = result.leftTurretDeg; // both sides are identical

    // First-order exponential lag: how far we'd *like* to move this tick
    double tau = SimPhysicsConstants.kTurretSimTauSeconds;
    double alpha = 1.0 - Math.exp(-dt / tau);
    double desiredDelta = (targetAngleDeg - simTurretAngleDeg) * alpha;

    // Clamp velocity so the turret can't slew faster than the motor allows
    double maxDelta = SimPhysicsConstants.kTurretSimMaxVelocityDegPerSec * dt;
    desiredDelta = Math.max(-maxDelta, Math.min(maxDelta, desiredDelta));

    simTurretAngleDeg += desiredDelta;

    // Hard-stop at physical turret limits
    simTurretAngleDeg =
        Math.max(
            ScorerConstants.kTurretMinPositionUnits,
            Math.min(ScorerConstants.kTurretMaxPositionUnits, simTurretAngleDeg));

    // Pack into IO inputs struct (auto-logged by AdvantageKit)
    // Apply turret command deadband — if the new aim is within kTurretDeadbandDeg
    // of the previous output, hold the previous value to suppress jitter.
    double deadband = ScorerConstants.kTurretDeadbandDeg;
    double outLeftTurret =
        Math.abs(result.leftTurretDeg - prevLeftTurretDeg) < deadband
            ? prevLeftTurretDeg
            : result.leftTurretDeg;
    double outRightTurret =
        Math.abs(result.rightTurretDeg - prevRightTurretDeg) < deadband
            ? prevRightTurretDeg
            : result.rightTurretDeg;
    prevLeftTurretDeg = outLeftTurret;
    prevRightTurretDeg = outRightTurret;

    inputs.leftTurretAngleDeg = outLeftTurret;
    inputs.leftHoodAngleDeg = result.leftHoodDeg;
    inputs.leftFeederRPM = result.leftFeederRPM;
    inputs.rightTurretAngleDeg = outRightTurret;
    inputs.rightHoodAngleDeg = result.rightHoodDeg;
    inputs.rightFeederRPM = result.rightFeederRPM;

    inputs.allianceColor = String.valueOf(alliance);

    inputs.targetXMeters = result.target.getX();
    inputs.targetYMeters = result.target.getY();

    inputs.robotXMeters = pose.getX();
    inputs.robotYMeters = pose.getY();
    inputs.robotHeadingDeg = pose.getRotation().getDegrees();

    inputs.distanceToTargetMeters = pose.getTranslation().getDistance(result.target);

    // Log with AdvantageKit so values appear in AdvantageScope under "TurretAim/"
    Logger.processInputs("TurretAim", inputs);

    if (shouldLog) {
      // Also log the six key values at the top level for quick graphing
      Logger.recordOutput("TurretAim/LeftTurretDeg", result.leftTurretDeg);
      Logger.recordOutput("TurretAim/LeftHoodDeg", result.leftHoodDeg);
      Logger.recordOutput("TurretAim/LeftFeederRPM", result.leftFeederRPM);
      Logger.recordOutput("TurretAim/RightTurretDeg", result.rightTurretDeg);
      Logger.recordOutput("TurretAim/RightHoodDeg", result.rightHoodDeg);
      Logger.recordOutput("TurretAim/RightFeederRPM", result.rightFeederRPM);
      Logger.recordOutput("TurretAim/DistToTarget", inputs.distanceToTargetMeters);

      // ---- TOF aim-ahead logging ----
      Logger.recordOutput("TurretAim/ConvergedTofSeconds", convergedTof);
      Logger.recordOutput("TurretAim/PhaseDelaySeconds", phaseDelay);
      Logger.recordOutput("TurretAim/ReleaseDelaySeconds", releaseDelay);
      Logger.recordOutput("TurretAim/PhaseCorrectedPose", phaseCorrectedPose);
      Logger.recordOutput("TurretAim/ReleasePose", releasePose);
      Logger.recordOutput("TurretAim/BallisticPose", ballisticPose);
      Logger.recordOutput("TurretAim/FieldVelocityMps", Math.hypot(fieldVx, fieldVy));
      Logger.recordOutput("TurretAim/Home", home);

      // ---- Instant aim line (where calculator WANTS to aim) ----
      Translation2d targetXY = result.target;
      double dx = targetXY.getX() - pose.getX();
      double dy = targetXY.getY() - pose.getY();
      Rotation2d bearing = new Rotation2d(Math.atan2(dy, dx));

      Pose2d[] aimLine =
          new Pose2d[] {new Pose2d(pose.getTranslation(), bearing), new Pose2d(targetXY, bearing)};
      Logger.recordOutput("TurretAim/AimLine", aimLine);

      // Target ghost on the field (instant)
      Logger.recordOutput("TurretAim/TargetPose", new Pose2d(targetXY, bearing));

      // ---- Simulated turret ghost (with inertia) ----
      // Pose2d at the robot position whose rotation = robotHeading + simTurretAngle.
      // 0° turret = facing robot front, so the ghost spins on the robot.
      Rotation2d simFieldBearing =
          pose.getRotation().plus(Rotation2d.fromDegrees(simTurretAngleDeg));
      Logger.recordOutput(
          "TurretAim/SimTurretPose", new Pose2d(pose.getTranslation(), simFieldBearing));

      // Also log the raw sim angle for graphing alongside the commanded angle
      Logger.recordOutput("TurretAim/SimTurretAngleDeg", simTurretAngleDeg);
      Logger.recordOutput("TurretAim/CommandedTurretAngleDeg", targetAngleDeg);
      Logger.recordOutput("TurretAim/LockedOn", isLockedOn());
    }
  }

  // ---- Accessors for other subsystems ----

  /**
   * @return The latest aim result, or null if periodic() hasn't run yet.
   */
  public TurretAimCalculator.AimResult getLatestResult() {
    return latestResult;
  }

  public double getLeftTurretAngleDeg() {
    return inputs.leftTurretAngleDeg;
  }

  public double getLeftHoodAngleDeg() {
    return inputs.leftHoodAngleDeg;
  }

  public double getRightTurretAngleDeg() {
    return inputs.rightTurretAngleDeg;
  }

  public double getRightHoodAngleDeg() {
    return inputs.rightHoodAngleDeg;
  }

  public double getLeftFeederRPM() {
    return inputs.leftFeederRPM;
  }

  public double getRightFeederRPM() {
    return inputs.rightFeederRPM;
  }

  /** Returns the simulated (lagged) turret angle in degrees, for visualization. */
  public double getSimTurretAngleDeg() {
    return simTurretAngleDeg;
  }

  /**
   * Returns true when the simulated turret position is within the lock-on tolerance of the
   * commanded angle. Use this to gate feeders / shooting — don't blow balls until the turret is
   * actually pointed at the target.
   */
  public boolean isLockedOn() {
    if (latestResult == null) return false;
    double error = Math.abs(simTurretAngleDeg - latestResult.leftTurretDeg);
    return error <= ScorerConstants.kTurretLockOnToleranceDeg;
  }

  // ---- Helpers ----

  /**
   * Returns 'B' or 'R' based on DriverStation alliance data. Defaults to 'B' if unknown (sim
   * startup).
   */
  private static char getAllianceChar() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 'R' : 'B';
  }
}
