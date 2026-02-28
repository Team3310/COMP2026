package frc.robot.Auton;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.RobotContainer;

public class Forward2m extends AutonCommandBase {
  public Forward2m(RobotContainer robotContainer) {
    super(robotContainer, Rotation2d.fromDegrees(0));

    this.addCommands(followPath(Paths.forward2m));
  }
}
