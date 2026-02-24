package frc.robot.subsystems.scorer.turret;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    // Run the aim calculator
    TurretAimCalculator.AimResult result = TurretAimCalculator.calculate(pose);
    latestResult = result;

    // Pack into IO inputs struct (auto-logged by AdvantageKit)
    inputs.leftTurretAngleDeg = result.leftTurretDeg;
    inputs.leftHoodAngleDeg = result.leftHoodDeg;
    inputs.rightTurretAngleDeg = result.rightTurretDeg;
    inputs.rightHoodAngleDeg = result.rightHoodDeg;

    inputs.allianceColor = String.valueOf(alliance);

    inputs.targetXMeters = result.target.getX();
    inputs.targetYMeters = result.target.getY();

    inputs.robotXMeters = pose.getX();
    inputs.robotYMeters = pose.getY();
    inputs.robotHeadingDeg = pose.getRotation().getDegrees();

    inputs.distanceToTargetMeters = pose.getTranslation().getDistance(result.target);

    // Log with AdvantageKit so values appear in AdvantageScope under "TurretAim/"
    Logger.processInputs("TurretAim", inputs);

    // Also log the four key values at the top level for quick graphing
    Logger.recordOutput("TurretAim/LeftTurretDeg", result.leftTurretDeg);
    Logger.recordOutput("TurretAim/LeftHoodDeg", result.leftHoodDeg);
    Logger.recordOutput("TurretAim/RightTurretDeg", result.rightTurretDeg);
    Logger.recordOutput("TurretAim/RightHoodDeg", result.rightHoodDeg);
    Logger.recordOutput("TurretAim/DistToTarget", inputs.distanceToTargetMeters);
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

  // ---- Helpers ----

  /**
   * Returns 'B' or 'R' based on DriverStation alliance data. Defaults to 'B' if unknown (sim
   * startup).
   */
  private static char getAllianceChar() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 'R' : 'B';
  }
}
