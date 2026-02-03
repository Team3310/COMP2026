package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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

  // -------------------- Position Control Commands --------------------

  /**
   * Command to run pivot at target position.
   *
   * @return Command that sets position as angle
   */
  public Command setDegreesCommand(double position) {
    return motionMagicSetpointCommand(() -> Units.degreesToRotations(position))
        .withName("Intake Pivot Maintain Setpoint");
  }

  @Override
  public void periodic() {
    super.periodic();
  }
}
