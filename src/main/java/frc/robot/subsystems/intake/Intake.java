// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  // Default speeds (0.0 to 1.0)
  private static final double DEFAULT_FORWARD_SPEED = 0.6;
  private static final double DEFAULT_BACKWARD_SPEED = 0.5;
  private static final double MAX_VOLTAGE = 12.0;

  /** Creates a new Intake subsystem. */
  public Intake(IntakeIO io) {
    this.io = io;

    // Put speed controls on dashboard
    SmartDashboard.putNumber("Intake/ForwardSpeed", DEFAULT_FORWARD_SPEED);
    SmartDashboard.putNumber("Intake/BackwardSpeed", DEFAULT_BACKWARD_SPEED);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  /** Run intake forward (positive voltage). */
  public void runForward() {
    double speed = SmartDashboard.getNumber("Intake/ForwardSpeed", DEFAULT_FORWARD_SPEED);
    io.setVoltage(speed * MAX_VOLTAGE);
  }

  /** Run intake backward (negative voltage). */
  public void runBackward() {
    double speed = SmartDashboard.getNumber("Intake/BackwardSpeed", DEFAULT_BACKWARD_SPEED);
    io.setVoltage(-speed * MAX_VOLTAGE);
  }

  /** Stop the intake motor. */
  public void stop() {
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
