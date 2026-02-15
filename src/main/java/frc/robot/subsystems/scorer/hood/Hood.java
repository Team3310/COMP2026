package frc.robot.subsystems.scorer.hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.*;

/**
 * The {@code IntakePivotSubsystem} controls the pivoting mechanism of the robot's intake. It
 * manages the deployment and stowing of the intake using a motor for precise position control and
 * feedback.
 */
public class Hood extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  public MotorIO motorIO;

  public Hood(final ServoMotorSubsystemConfig motorConfig, final MotorIO motorIO) {
    super(motorConfig, new MotorInputsAutoLogged(), motorIO);
    this.motorIO = motorIO;
  }

  public void setTeleopDefaultCommand() {
    this.getDefaultCommand().cancel();
    this.setDefaultCommand(
        motionMagicSetpointCommand(this::getPositionSetpointUnits)
            .withName("Hood Maintain Setpoint (default)")
            .ignoringDisable(true));
  }

  // -------------------- Position Control Commands --------------------

  /**
   * Command to run pivot at target position.
   *
   * @return Command that sets position as angle
   */
  public Command setDegreesCommand(double position) {
    return motionMagicSetpointCommand(() -> (position)).withName("Hood Maintain Setpoint");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Hood Position", getCurrentPosition());
    super.periodic();
  }
}
