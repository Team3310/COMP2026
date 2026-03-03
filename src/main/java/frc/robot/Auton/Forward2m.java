package frc.robot.Auton;

import frc.robot.RobotContainer;

public class Forward2m extends AutonCommandBase {
  public Forward2m(RobotContainer robotContainer) {
    super(robotContainer, robotContainer.getDrive().getPose());
    this.addCommands(followPath(Paths.forward2m));
  }
}
