package frc.robot.Auton.Dep;

import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class DepToSSTrench1 extends AutonCommandBase {
  public DepToSSTrench1(RobotContainer robotContainer) {
    super(robotContainer, robotContainer.getDrive().getPose());

    this.addCommands(followPath(Paths.depToSSTrench));
  }
}
