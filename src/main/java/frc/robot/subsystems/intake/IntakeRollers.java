package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.ServoMotorSubsystem;
import frc.lib.subsystems.ServoMotorSubsystemConfig;
import frc.robot.Constants;

/**
 * The {@code IntakeRollerSubsystem} controls the roller mechanism of the robot's intake. It manages
 * the speed and direction of the intake rollers to collect and feed game pieces.
 */
public class IntakeRollers extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  public MotorIO motorIO;

  // Default velocity setpoints (units per second - tune these values)

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

  // -------------------- Velocity Control Commands --------------------

  /**
   * Command to run intake at target velocity for collecting game pieces.
   *
   * @return Command that runs intake forward
   */
  public Command intakeCommand() {
    return velocitySetpointCommand(() -> Constants.IntakeConstants.kIntakeVelocityInRPM)
        .withName("Intake Forward");
  }

  /**
   * Command to run intake at target velocity for collecting game pieces.
   *
   * @return Command that runs intake backward
   */
  public Command outakeCommand() {
    return velocitySetpointCommand(() -> Constants.IntakeConstants.kIntakeVelocityOutRPM)
        .withName("Intake Forward");
  }
}
