package frc.robot.subsystems.Scorer.flywheel;

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
public class Flywheel extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  public MotorIO motorIO;

  // Default velocity setpoints (units per second - tune these values)

  public Flywheel(final ServoMotorSubsystemConfig motorConfig, final MotorIO motorIO) {
    super(motorConfig, new MotorInputsAutoLogged(), motorIO);
    this.motorIO = motorIO;
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
  public Command forwardCommand() {
    return velocitySetpointCommand(() -> Constants.ScorerConstants.kShootRPM)
        .withName("Flywheel Forward");
  }

  /**
   * Command to run intake at target velocity for collecting game pieces.
   *
   * @return Command that runs intake backward
   */
  public Command backwardCommand() {
    return velocitySetpointCommand(() -> Constants.ScorerConstants.kReverseRPM)
        .withName("Flywheel Reverse");
  }
}
