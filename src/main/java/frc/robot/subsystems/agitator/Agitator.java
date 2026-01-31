// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.subsystems.agitator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.velocity.VelocityControlIO;
import frc.robot.subsystems.velocity.VelocityControlIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

/**
 * Hopper subsystem for controlling game piece hopper. Uses AdvantageKit IO layer pattern for
 * hardware abstraction.
 */
public class Agitator extends SubsystemBase {
  private final VelocityControlIO io;
  private final VelocityControlIOInputsAutoLogged inputs = new VelocityControlIOInputsAutoLogged();

  // Default RPM setpoints - change these constants to tune
  private static final double DEFAULT_OUTTAKE_RPM = 1000.0;
  private static final double DEFAULT_INTAKE_RPM = 1000.0;

  // Feedforward constants - tune based on motor characterization
  private static final double DEFAULT_KS = 0.0; // Volts to overcome static friction
  private static final double DEFAULT_KV = 0.11; // Volts per rad/s

  // PID constants - tune for velocity control
  private static final double DEFAULT_KP = 0.6;
  private static final double DEFAULT_KI = 0.0;
  private static final double DEFAULT_KD = 0.0;

  // Tunable setpoints and gains
  private double backwardRPM = DEFAULT_OUTTAKE_RPM;
  private double forwardRPM = DEFAULT_INTAKE_RPM;
  private double kS = DEFAULT_KS;
  private double kV = DEFAULT_KV;
  private double kP = DEFAULT_KP;
  private double kI = DEFAULT_KI;
  private double kD = DEFAULT_KD;

  // Track last PID values to avoid spamming CAN bus
  private double lastKP = DEFAULT_KP;
  private double lastKI = DEFAULT_KI;
  private double lastKD = DEFAULT_KD;
  private double lastKV = DEFAULT_KV;
  private double lastKS = DEFAULT_KS;

  /** Creates a new Hopper subsystem. */
  public Agitator(VelocityControlIO io) {
    this.io = io;

    // Configure initial PID
    io.configurePID(kP, kI, kD, kV, kS);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    // Update calculated values
    inputs.currentRPM = inputs.velocityRadsPerSec * 60.0 / (2.0 * Math.PI);

    // Update tunable parameters in inputs for logging
    inputs.backwardRPM = backwardRPM;
    inputs.forwardRPM = forwardRPM;
    inputs.kP = kP;
    inputs.kI = kI;
    inputs.kD = kD;
    inputs.kS = kS;
    inputs.kV = kV;

    // Update PID gains ONLY if changed (avoid spamming CAN bus)
    if (kP != lastKP || kI != lastKI || kD != lastKD || kV != lastKV || kS != lastKS) {
      io.configurePID(kP, kI, kD, kV, kS);
      inputs.pidUpdated = true;
      lastKP = kP;
      lastKI = kI;
      lastKD = kD;
      lastKV = kV;
      lastKS = kS;
    } else {
      inputs.pidUpdated = false;
    }

    // Process all inputs - @AutoLog will handle logging everything
    Logger.processInputs("Hopper", inputs);
  }

  /** Run hopper forward at target RPM (for outtaking game pieces). */
  public void runForward() {
    double targetRadsPerSec = backwardRPM * 2.0 * Math.PI / 60.0;
    double ffVolts = Math.signum(targetRadsPerSec) * kS + kV * targetRadsPerSec;

    // Update inputs for @AutoLog
    inputs.targetRPM = backwardRPM;
    inputs.targetRadsPerSec = targetRadsPerSec;
    inputs.feedforwardVolts = ffVolts;
    inputs.commandState = "OUTTAKE";

    io.setVelocity(targetRadsPerSec, ffVolts);
  }

  /** Run hopper backward at target RPM (for intaking game pieces). */
  public void runBackward() {
    double targetRadsPerSec = -forwardRPM * 2.0 * Math.PI / 60.0; // Negative for backward
    double ffVolts = Math.signum(targetRadsPerSec) * kS + kV * targetRadsPerSec;

    // Update inputs for @AutoLog
    inputs.targetRPM = -forwardRPM;
    inputs.targetRadsPerSec = targetRadsPerSec;
    inputs.feedforwardVolts = ffVolts;
    inputs.commandState = "HOPPER";

    io.setVelocity(targetRadsPerSec, ffVolts);
  }

  /** Stop the hopper motor. */
  public void stop() {
    inputs.commandState = "STOPPED";
    io.stop();
  }

  /** Returns true if the hopper motor is connected. */
  public boolean isConnected() {
    return inputs.connected;
  }

  /** Returns the current draw of the hopper motor in amps. */
  public double getCurrentAmps() {
    return inputs.currentAmps;
  }

  // -------------------- Tuning Setters --------------------

  /** Set outtake RPM (use in test mode or via commands). */
  public void setBackwardRPM(double rpm) {
    this.backwardRPM = rpm;
  }

  /** Set hopper RPM (use in test mode or via commands). */
  public void setForwardRPM(double rpm) {
    this.forwardRPM = rpm;
  }

  /** Set PID gains (use in test mode or via commands). */
  public void setPID(double kP, double kI, double kD, double kV, double kS) {
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    this.kV = kV;
    this.kS = kS;
  }

  // -------------------- Commands --------------------

  /** Command to run hopper forward continuously. */
  public Command forwardCommand() {
    return startEnd(this::runForward, this::stop).withName("HopperForward");
  }

  /** Command to run hopper backward continuously. */
  public Command backwardCommand() {
    return startEnd(this::runBackward, this::stop).withName("HopperBackward");
  }

  /** Command to stop hopper. */
  public Command stopCommand() {
    return runOnce(this::stop).withName("StopHopper");
  }
}
