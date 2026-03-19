package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class OutToOut extends AutonCommandBase {
  public OutToOut(RobotContainer robotContainer) {
    super(robotContainer, Rotation2d.fromDegrees(180.0));

    this.addCommands(followPathAndSnowblow(Paths.OutToOut));
  }
}
