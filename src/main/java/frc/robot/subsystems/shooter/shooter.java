// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.velocity.VelocityControlIO;
import frc.robot.subsystems.velocity.VelocityControlIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem for controlling game piece intake. Uses AdvantageKit IO layer pattern for
 * hardware abstraction.
 */
public class shooter extends SubsystemBase {
  private final VelocityControlIO io;
  private final VelocityControlIOInputsAutoLogged inputs = new VelocityControlIOInputsAutoLogged();
  private final Module[] modules = new Module[3]; //shooter, feeder, and hood motors
  // Default RPM setpoints - change these constants to tune
  private static final double DEFAULT_OUTTAKE_RPM = 2000.0;
  private static final double DEFAULT_INTAKE_RPM = 3000.0;

  // Tunable setpoints and gains
  private double backwardRPM = DEFAULT_OUTTAKE_RPM;
  private double forwardRPM = DEFAULT_INTAKE_RPM;
  private double mainKv = 0.12;
  private double mainKp = 0.6;

  /** Creates a new Intake subsystem. */
  public shooter(
    VelocityControlIO io,
    ModuleIO shooterIO,
    ModuleIO feeder) {
    this.io = io;
      modules[0] = new Module(shooterIO);
      modules[1] = new Module(feederIO);
      modules[2] = new Module(hoodIO);

    // Configure initial PID
    io.configurePID(mainKp, 0.0, 0.0, mainKv, 0.0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Update calculated values
    inputs.currentRPM = inputs.velocityRadsPerSec * 60.0 / (2.0 * Math.PI);

    // Update tunable parameters in inputs for logging
    inputs.backwardRPM = backwardRPM;
    inputs.forwardRPM = forwardRPM;
    inputs.kP = mainKp;
    inputs.kI = kI;
    inputs.kD = kD;
    inputs.kS = mainKs;
    inputs.kV = mainKv;

    // Update PID gains ONLY if changed (avoid spamming CAN bus)
    if (mainKp != lastKP || kI != lastKI || kD != lastKD || mainKv != lastKV || mainKs != lastKS) {
      io.configurePID(mainKp, kI, kD, mainKv, mainKs);
      inputs.pidUpdated = true;
      lastKP = mainKp;
      lastKI = kI;
      lastKD = kD;
      lastKV = mainKv;
      lastKS = mainKs;
    } else {
      inputs.pidUpdated = false;
    }

    // Process all inputs - @AutoLog will handle logging everything
    Logger.processInputs("Intake", inputs);
  }

  /** Run intake forward at target RPM (for outtaking game pieces). */
  public void runForward() {
    double targetRadsPerSec = backwardRPM * 2.0 * Math.PI / 60.0;
    double ffVolts = Math.signum(targetRadsPerSec) * mainKs + mainKv * targetRadsPerSec;

    // Update inputs for @AutoLog
    inputs.targetRPM = backwardRPM;
    inputs.targetRadsPerSec = targetRadsPerSec;
    inputs.feedforwardVolts = ffVolts;
    inputs.commandState = "OUTTAKE";

    io.setVelocity(targetRadsPerSec, ffVolts);
  }

  /** Run intake backward at target RPM (for intaking game pieces). */
  public void runBackward() {
    double targetRadsPerSec = -forwardRPM * 2.0 * Math.PI / 60.0; // Negative for backward
    double ffVolts = Math.signum(targetRadsPerSec) * mainKs + mainKv * targetRadsPerSec;

    // Update inputs for @AutoLog
    inputs.targetRPM = -forwardRPM;
    inputs.targetRadsPerSec = targetRadsPerSec;
    inputs.feedforwardVolts = ffVolts;
    inputs.commandState = "INTAKE";

    io.setVelocity(targetRadsPerSec, ffVolts);
  }

  /** Stop the intake motor. */
  public void stop() {
    inputs.commandState = "STOPPED";
    io.stop();
  }

  /** Returns true if the intake motor is connected. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /** Returns the current draw of the intake motor in amps. */
  public double getCurrentAmps() {
    return inputs.currentAmps;
  }

  // -------------------- Tuning Setters --------------------

  /** Set outtake RPM (use in test mode or via commands). */
  public void setBackwardRPM(double rpm) {
    this.backwardRPM = rpm;
  }

  /** Set intake RPM (use in test mode or via commands). */
  public void setForwardRPM(double rpm) {
    this.forwardRPM = rpm;
  }

  /** Set PID gains (use in test mode or via commands). */
  public void setPID(double kP, double kI, double kD, double kV, double kS) {
    this.mainKp = kP;
    this.kI = kI;
    this.kD = kD;
    this.mainKv = kV;
    this.mainKs = kS;
  }

  // -------------------- Commands --------------------

  /** Command to run intake forward continuously. */
  public Command forwardCommand() {
    return startEnd(this::runForward, this::stop).withName("IntakeForward");
  }

  /** Command to run intake backward continuously. */
  public Command backwardCommand() {
    return startEnd(this::runBackward, this::stop).withName("IntakeBackward");
  }

  /** Command to stop intake. */
  public Command stopCommand() {
    return runOnce(this::stop).withName("StopIntake");
  }
}
