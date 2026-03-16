package frc.robot.subsystems.roof;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.*;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/**
 * The {@code Roof} subsystem controls a single servo motor on the robot's roof mechanism. It uses
 * Motion Magic position control via the shared {@link ServoMotorSubsystem} base class.
 *
 * <p>The motor is connected directly to the roboRIO CAN bus (not the CANivore).
 */
public class Roof extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  private static final int DASHBOARD_INTERVAL = 10; // ~5 Hz at a 20 ms main loop

  public MotorIO motorIO;
  private int dashboardCounter = 0;

  public Roof(final ServoMotorSubsystemConfig motorConfig, final MotorIO motorIO) {
    super(motorConfig, new MotorInputsAutoLogged(), motorIO);
    this.motorIO = motorIO;
    this.setCurrentPosition(Constants.RoofConstants.kRoofStowedDegrees);
    this.positionSetpointUnits = Constants.RoofConstants.kRoofStowedDegrees;
  }

  /**
   * Sets the default command to continuously hold the last-commanded position via Motion Magic.
   * Call this in teleop init so the roof doesn't go limp when a position command finishes.
   */
  public void setTeleopDefaultCommand() {
    this.getDefaultCommand().cancel();
    this.setDefaultCommand(
        motionMagicSetpointCommand(this::getPositionSetpointUnits)
            .withName("Roof Maintain Setpoint (default)")
            .ignoringDisable(true));
  }

  // -------------------- Position Control Commands --------------------

  /**
   * Command to move the roof to a fixed position in degrees.
   *
   * @param position target position in degrees
   * @return Motion Magic position command
   */
  public Command setDegreesCommand(double position) {
    return motionMagicSetpointCommand(() -> position).withName("Roof Set Degrees");
  }

  /**
   * Command to continuously track a supplied position in degrees.
   *
   * @param position supplier providing the target position in degrees
   * @return Motion Magic position command
   */
  public Command setDegreesCommand(DoubleSupplier position) {
    return motionMagicSetpointCommand(position).withName("Roof Track Degrees");
  }

  /** Command to move the roof to its maximum position. */
  public Command setMaxCommand() {
    return motionMagicSetpointCommand(() -> Constants.RoofConstants.kRoofMaxDegrees);
  }

  /** Command to stow the roof (move to minimum/stowed position). */
  public Command stowCommand() {
    return motionMagicSetpointCommand(() -> Constants.RoofConstants.kRoofStowedDegrees);
  }

  @Override
  public void periodic() {
    if (++dashboardCounter % DASHBOARD_INTERVAL == 0) {
      SmartDashboard.putNumber(getName() + "/Position", getCurrentPosition());
    }
    super.periodic();
  }
}
