package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.ServoMotorSubsystemWithFollowers;
import frc.lib.subsystems.ServoMotorSubsystemWithFollowersConfig;
import frc.robot.Constants;

/**
 * The {@code IntakeRollerSubsystem} controls the roller mechanism of the robot's intake. It manages
 * the speed and direction of the intake rollers to collect and feed game pieces.
 */
public class IntakeRollers
    extends ServoMotorSubsystemWithFollowers<MotorInputsAutoLogged, MotorIO> {
  public MotorIO motorIO;
  // Default velocity setpoints (units per second - tune these values)

  public IntakeRollers(
      final ServoMotorSubsystemWithFollowersConfig leadConfig,
      final MotorIO leadIO,
      final MotorIO[] followerIO) {

    super(
        leadConfig,
        new MotorInputsAutoLogged(),
        leadIO,
        new MotorInputsAutoLogged[] {new MotorInputsAutoLogged()},
        followerIO);
    motorIO = leadIO;
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
    return velocitySetpointCommand(() -> Constants.IntakeConstants.kIntakeVelocityRPM)
        .withName("Intake Forward");
  }

  /**
   * Command to run intake at target velocity for collecting game pieces.
   *
   * @return Command that runs intake backward
   */
  public Command outakeCommand() {
    return velocitySetpointCommand(() -> Constants.IntakeConstants.kOutakeVelocityRPM)
        .withName("Intake Forward");
  }
}
