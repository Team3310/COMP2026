package frc.robot.Auton.Dep;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class DepToSSTrench1 extends AutonCommandBase {
  public DepToSSTrench1(RobotContainer robotContainer) {
    super(robotContainer, robotContainer.getDrive().getPose());

    this.addCommands(
        new SequentialCommandGroup(
            followPath(Paths.depToSSTrench), followPathAndSnowblow(Paths.trenchToDepo)));
  }
}
