// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;

/** IntakeIO implementation for TalonFX motor controller. Uses Phoenix 6 API for motor control. */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX motor;

  // Control requests
  private final VoltageOut voltageControl = new VoltageOut(0).withEnableFOC(true);
  private final VelocityVoltage velocityControl = new VelocityVoltage(0).withEnableFOC(true);

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> current;
  private final StatusSignal<Temperature> temp;

  public IntakeIOTalonFX(int canId, CANBus canBus) {
    motor = new TalonFX(canId, canBus);

    // Configure motor
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = 40.0;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Apply configuration
    motor.getConfigurator().apply(config);

    // Set up status signals
    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    current = motor.getStatorCurrent();
    temp = motor.getDeviceTemp();

    // Set update frequencies (Hz)
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, current, temp);

    // Optimize bus utilization
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    // Refresh all signals
    BaseStatusSignal.refreshAll(position, velocity, appliedVolts, current, temp);

    inputs.connected = BaseStatusSignal.isAllGood(position, velocity, appliedVolts, current, temp);
    inputs.positionRads = position.getValueAsDouble() * 2.0 * Math.PI;
    inputs.velocityRadsPerSec = velocity.getValueAsDouble() * 2.0 * Math.PI;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = current.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageControl.withOutput(volts));
  }

  @Override
  public void setVelocity(double velocityRadsPerSec, double ffVolts) {
    motor.setControl(
        velocityControl
            .withVelocity(velocityRadsPerSec / (2.0 * Math.PI))
            .withFeedForward(ffVolts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    motor.getConfigurator().apply(config.Slot0);
  }
}
