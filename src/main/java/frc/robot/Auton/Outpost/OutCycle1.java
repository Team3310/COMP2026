package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class OutCycle1 extends AutonCommandBase {
  public OutCycle1(RobotContainer robotContainer) {
    super(robotContainer, Rotation2d.fromDegrees(180.0));

    this.addCommands(followPathAndSnowblow(Paths.OutCycle));
  }
}
