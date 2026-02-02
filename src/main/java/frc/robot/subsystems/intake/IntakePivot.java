package frc.robot.subsystems.intake;

import frc.lib.subsystems.*;
import frc.robot.Constants;

/**
 * The {@code IntakePivotSubsystem} controls the pivoting mechanism of the robot's intake. It
 * manages the deployment and stowing of the intake using a motor for precise position control and
 * feedback.
 */
public class IntakePivot extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  public MotorIO motorIO;

  public IntakePivot(final ServoMotorSubsystemConfig motorConfig, final MotorIO motorIO) {
    super(motorConfig, new MotorInputsAutoLogged(), motorIO);
    this.motorIO = motorIO;

    this.setCurrentPosition(Constants.IntakeConstants.kIntakePivotStowPositionRadians);
    this.positionSetpointUnits = Constants.IntakeConstants.kIntakePivotStowPositionRadians;
  }

  public void setTeleopDefaultCommand() {
    this.getDefaultCommand().cancel();
    this.setDefaultCommand(
        motionMagicSetpointCommand(this::getPositionSetpointUnits)
            .withName("Intake Pivot Maintain Setpoint (default)")
            .ignoringDisable(true));
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
