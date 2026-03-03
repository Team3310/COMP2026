package frc.robot.Auton;

import frc.robot.RobotContainer;

public class OneAuton extends AutonCommandBase {
  public OneAuton(RobotContainer robotContainer) {
    super(robotContainer, robotContainer.getDrive().getPose());

    this.addCommands(followPath(Paths.forward2m));
  }
}
