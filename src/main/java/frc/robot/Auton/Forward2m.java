package frc.robot.Auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;

public class Forward2m extends AutonCommandBase {
  private static final double kForwardSpeedMetersPerSecond = 1.0;
  private static final double kDriveTimeSeconds = 2.0;

  public Forward2m(RobotContainer robotContainer) {
    super(robotContainer, (Pose2d) null);

    this.addCommands(
        Commands.run(
                () ->
                    robotContainer
                        .getDrive()
                        .runVelocity(new ChassisSpeeds(kForwardSpeedMetersPerSecond, 0.0, 0.0)),
                robotContainer.getDrive())
            .withTimeout(kDriveTimeSeconds)
            .finallyDo(robotContainer.getDrive()::stop));
  }
}
