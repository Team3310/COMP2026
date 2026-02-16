package frc.robot.subsystems.scorer.turret;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.subsystems.*;
import frc.robot.Constants;

/** The {@code Turret} controls the pivoting mechanism of the robot's launcher turret. */
public class Turret extends ServoMotorSubsystem<MotorInputsAutoLogged, MotorIO> {
  public MotorIO motorIO;

  public Turret(final ServoMotorSubsystemConfig motorConfig, final MotorIO motorIO) {
    super(motorConfig, new MotorInputsAutoLogged(), motorIO);
    this.motorIO = motorIO;
    this.setCurrentPosition(Constants.ScorerConstants.kTurretStowedPosition);
    this.positionSetpointUnits = Constants.ScorerConstants.kTurretStowedPosition;
  }

  public void setTeleopDefaultCommand() {
    this.getDefaultCommand().cancel();
    this.setDefaultCommand(
        motionMagicSetpointCommand(this::getPositionSetpointUnits)
            .withName("Turret Maintain Setpoint (default)")
            .ignoringDisable(true));
  }

  // -------------------- Position Control Commands --------------------

  /**
   * Command to run turret at target position.
   *
   * @return Command that sets position as angle
   */
  public Command setDegreesCommand(double position) {
    return motionMagicSetpointCommand(() -> (position)).withName("Turret Maintain Setpoint");
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret Position", getCurrentPosition());
    super.periodic();
  }
}
