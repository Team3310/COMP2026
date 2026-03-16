package frc.robot.subsystems.scorer.hood;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.*;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;

/** The {@code Hood} controls the pivoting mechanism of the robot's launcher hood. */
public class Hood extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  private static final int DASHBOARD_INTERVAL = 10; // ~5 Hz at a 20 ms main loop

  public MotorIO motorIO;
  private int dashboardCounter = 0;

  public Hood(final ServoMotorSubsystemConfig motorConfig, final MotorIO motorIO) {
    super(motorConfig, new MotorInputsAutoLogged(), motorIO);
    this.motorIO = motorIO;
    this.setCurrentPosition(Constants.ScorerConstants.kHoodStowedDegrees);
    this.positionSetpointUnits = Constants.ScorerConstants.kHoodStowedDegrees;
  }

  public void setTeleopDefaultCommand() {
    this.getDefaultCommand().cancel();
    this.setDefaultCommand(
        motionMagicSetpointCommand(this::getPositionSetpointUnits)
            .withName("Hood Maintain Setpoint (default)")
            .ignoringDisable(true));
  }

  // -------------------- Torque Current Control Command --------------------

  public Command setTorqueCurrentFOCCommand(double torqueCurrent) {
    return setTorqueCurrentFOC(() -> torqueCurrent); // change to DoubleSupplier
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

  public Command setDegreesCommand(DoubleSupplier position) {
    return motionMagicSetpointCommand(position).withName("Hood Aim Tracking");
  }

  public Command setDegreesCommandRunEnd(double positionRun, double positionEnd) {
    return runEnd(
            () -> {
              setDegreesCommand(positionRun);
            },
            () -> {
              setDegreesCommand(positionEnd);
            })
        .withName(getName() + " VelocityControl");
  }

  public Command setMaxCommand() {
    return motionMagicSetpointCommand(() -> Constants.ScorerConstants.kHoodMaxDegrees);
  }

  @Override
  public void periodic() {
    if (++dashboardCounter % DASHBOARD_INTERVAL == 0) {
      SmartDashboard.putNumber(getName() + "/Position", getCurrentPosition());
    }
    super.periodic();
  }
}
