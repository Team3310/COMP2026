package frc.robot.subsystems.scorer.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
  private final Supplier<Pose2d> poseSupplier;
  private final TurretAimIOInputsAutoLogged inputs = new TurretAimIOInputsAutoLogged();

  // Cached latest result for external consumers
  private TurretAimCalculator.AimResult latestResult = null;

  // ---- Simulated turret inertia state ----
  // Tracks where the turret *physically is* (with lag), vs. where the
  // calculator *wants* it to be (instant).  Used purely for AdvantageScope
  // visualisation — the logged ghost Pose2d rotates with realistic inertia.
  private double simTurretAngleDeg = 0.0; // current simulated turret angle
  private double prevTimestamp = -1.0; // for dt calculation

  /**
   * @param poseSupplier Supplies the robot's current field pose (usually {@code drive::getPose}).
   */
  public TurretAimManager(Supplier<Pose2d> poseSupplier) {
    this.poseSupplier = poseSupplier;
  }

  @Override
  public void periodic() {
    Pose2d pose = poseSupplier.get();
    char alliance = getAllianceChar();

    // Run the aim calculator (instant — where the turret *wants* to be)
    TurretAimCalculator.AimResult result = TurretAimCalculator.calculate(pose);
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
    inputs.leftTurretAngleDeg = result.leftTurretDeg;
    inputs.leftHoodAngleDeg = result.leftHoodDeg;
    inputs.leftFeederRPM = result.leftFeederRPM;
    inputs.rightTurretAngleDeg = result.rightTurretDeg;
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

    // Also log the six key values at the top level for quick graphing
    Logger.recordOutput("TurretAim/LeftTurretDeg", result.leftTurretDeg);
    Logger.recordOutput("TurretAim/LeftHoodDeg", result.leftHoodDeg);
    Logger.recordOutput("TurretAim/LeftFeederRPM", result.leftFeederRPM);
    Logger.recordOutput("TurretAim/RightTurretDeg", result.rightTurretDeg);
    Logger.recordOutput("TurretAim/RightHoodDeg", result.rightHoodDeg);
    Logger.recordOutput("TurretAim/RightFeederRPM", result.rightFeederRPM);
    Logger.recordOutput("TurretAim/DistToTarget", inputs.distanceToTargetMeters);

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
    Rotation2d simFieldBearing = pose.getRotation().plus(Rotation2d.fromDegrees(simTurretAngleDeg));
    Logger.recordOutput(
        "TurretAim/SimTurretPose", new Pose2d(pose.getTranslation(), simFieldBearing));

    // Also log the raw sim angle for graphing alongside the commanded angle
    Logger.recordOutput("TurretAim/SimTurretAngleDeg", simTurretAngleDeg);
    Logger.recordOutput("TurretAim/CommandedTurretAngleDeg", targetAngleDeg);
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

  // ---- Helpers ----

  /**
   * Returns 'B' or 'R' based on DriverStation alliance data. Defaults to 'B' if unknown (sim
   * startup).
   */
  private static char getAllianceChar() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 'R' : 'B';
  }
}
