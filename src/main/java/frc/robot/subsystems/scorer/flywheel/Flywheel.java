package frc.robot.subsystems.scorer.flywheel;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.MotorIO;
import frc.lib.subsystems.MotorInputsAutoLogged;
import frc.lib.subsystems.ServoMotorSubsystem;
import frc.lib.subsystems.ServoMotorSubsystemConfig;
import frc.robot.Constants;

/**
 * The {@code Flywheel} controls the flywheel mechanism of the robot's launcher. It manages the
 * speed and direction of the flywheel to shoot game pieces.
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

  // -------------------- Torque Current Control Command --------------------

  public Command setTorqueCurrentFOCCommand(double torqueCurrent) {
    return setTorqueCurrentFOC(() -> torqueCurrent); // change to DoubleSupplier
  }

  // -------------------- Velocity Control Commands --------------------

  /**
   * Command to run flywheel at target velocity for collecting game pieces.
   *
   * @return Command that runs flywheel forward
   */
  public Command shootCommand() {
    return velocitySetpointCommand(() -> Constants.ScorerConstants.kShootRPM)
        .withName("Flywheel Forward");
  }

  public Command setRPMCommand(double rpm) {
    return velocitySetpointCommand(() -> rpm).withName("Flywheel Set RPM");
  }

  /**
   * Command to run flywheel at target velocity for collecting game pieces.
   *
   * @return Command that runs flywheel backward
   */
  public Command reverseCommand() {
    return velocitySetpointCommand(() -> Constants.ScorerConstants.kReverseShootRPM)
        .withName("Flywheel Reverse");
  }

  /**
   * Command to stop the flywheel.
   *
   * @return Command that stops the flywheel
   */
  public Command offCommand() {
    return velocitySetpointCommand(() -> 0.0).withName("Flywheel Off");
  }
}
