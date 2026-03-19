package frc.robot.Auton.Dep;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class DepCycle1 extends AutonCommandBase {
  public DepCycle1(RobotContainer robotContainer) {
    super(robotContainer, Rotation2d.fromDegrees(180.0));

    this.addCommands(followPathAndSnowblow(Paths.DepCycle));
  }
}
