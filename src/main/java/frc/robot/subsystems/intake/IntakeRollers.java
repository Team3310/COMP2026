package frc.robot.subsystems.intake;

import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.ServoMotorSubsystem;
import frc.lib.subsystems.ServoMotorSubsystemConfig;

/**
 * The {@code IntakeRollerSubsystem} controls the roller mechanism of the robot's intake. It manages
 * the speed and direction of the intake rollers to collect and feed game pieces.
 */
public class IntakeRollers extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  public MotorIO motorIO;

  public IntakeRollers(final ServoMotorSubsystemConfig motorConfig, final MotorIO motorIO) {
    super(motorConfig, new MotorInputsAutoLogged(), motorIO);
    this.motorIO = motorIO;
  }

  public double getPositionRotations() {
    return inputs.unitPosition;
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
