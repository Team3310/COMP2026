// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean connected = true;
    public double positionRads = 0.0;
    public double velocityRadsPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Run intake at specified voltage. */
  public default void setVoltage(double volts) {}

  /** Run intake at specified velocity in radians per second. */
  public default void setVelocity(double velocityRadsPerSec, double ffVolts) {}

  /** Stop the intake motor. */
  public default void stop() {}

  /** Configure PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}
}
