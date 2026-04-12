package frc.robot.Auton.Dep;

import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class DepMadtown extends AutonCommandBase {
  public DepMadtown(RobotContainer robotContainer) {
    super(robotContainer, null);

    this.addCommands(followPath(Paths.DepMadtown));
  }
}
