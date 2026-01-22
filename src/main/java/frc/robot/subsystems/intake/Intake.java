// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * Intake subsystem for controlling game piece intake. Uses AdvantageKit IO layer pattern for
 * hardware abstraction.
 */
public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // Default RPM setpoints - change these constants to tune
  private static final double DEFAULT_OUTTAKE_RPM = 2000.0;
  private static final double DEFAULT_INTAKE_RPM = 3000.0;

  // Feedforward constants - tune based on motor characterization
  private static final double DEFAULT_KS = 0.0; // Volts to overcome static friction
  private static final double DEFAULT_KV = 0.11; // Volts per rad/s

  // PID constants - tune for velocity control
  private static final double DEFAULT_KP = 0.6;
  private static final double DEFAULT_KI = 0.0;
  private static final double DEFAULT_KD = 0.0;

  // Tunable setpoints and gains
  private double outtakeRPM = DEFAULT_OUTTAKE_RPM;
  private double intakeRPM = DEFAULT_INTAKE_RPM;
  private double kS = DEFAULT_KS;
  private double kV = DEFAULT_KV;
  private double kP = DEFAULT_KP;
  private double kI = DEFAULT_KI;
  private double kD = DEFAULT_KD;

  // Track last PID values to avoid spamming CAN bus
  private double lastKP = DEFAULT_KP;
  private double lastKI = DEFAULT_KI;
  private double lastKD = DEFAULT_KD;

  /** Creates a new Intake subsystem. */
  public Intake(IntakeIO io) {
    this.io = io;

    // Configure initial PID
    io.configurePID(kP, kI, kD);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Update calculated values
    inputs.currentRPM = inputs.velocityRadsPerSec * 60.0 / (2.0 * Math.PI);

    // Update tunable parameters in inputs for logging
    inputs.outtakeRPM = outtakeRPM;
    inputs.intakeRPM = intakeRPM;
    inputs.kP = kP;
    inputs.kI = kI;
    inputs.kD = kD;
    inputs.kS = kS;
    inputs.kV = kV;

    // Update PID gains ONLY if changed (avoid spamming CAN bus)
    if (kP != lastKP || kI != lastKI || kD != lastKD) {
      io.configurePID(kP, kI, kD);
      inputs.pidUpdated = true;
      lastKP = kP;
      lastKI = kI;
      lastKD = kD;
    } else {
      inputs.pidUpdated = false;
    }

    // Process all inputs - @AutoLog will handle logging everything
    Logger.processInputs("Intake", inputs);
  }

  /** Run intake forward at target RPM (for outtaking game pieces). */
  public void runForward() {
    double targetRadsPerSec = outtakeRPM * 2.0 * Math.PI / 60.0;
    double ffVolts = Math.signum(targetRadsPerSec) * kS + kV * targetRadsPerSec;

    // Update inputs for @AutoLog
    inputs.targetRPM = outtakeRPM;
    inputs.targetRadsPerSec = targetRadsPerSec;
    inputs.feedforwardVolts = ffVolts;
    inputs.commandState = "OUTTAKE";

    io.setVelocity(targetRadsPerSec, ffVolts);
  }

  /** Run intake backward at target RPM (for intaking game pieces). */
  public void runBackward() {
    double targetRadsPerSec = -intakeRPM * 2.0 * Math.PI / 60.0; // Negative for backward
    double ffVolts = Math.signum(targetRadsPerSec) * kS + kV * targetRadsPerSec;

    // Update inputs for @AutoLog
    inputs.targetRPM = -intakeRPM;
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
  public void setOuttakeRPM(double rpm) {
    this.outtakeRPM = rpm;
  }

  /** Set intake RPM (use in test mode or via commands). */
  public void setIntakeRPM(double rpm) {
    this.intakeRPM = rpm;
  }

  /** Set PID gains (use in test mode or via commands). */
  public void setPID(double kP, double kI, double kD) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
  }

  /** Set feedforward gains (use in test mode or via commands). */
  public void setFeedforward(double kS, double kV) {
    this.kS = kS;
    this.kV = kV;
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
