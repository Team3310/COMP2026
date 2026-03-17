package frc.robot.Auton.Out;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class OutToSSTrench1 extends AutonCommandBase {
  public OutToSSTrench1(RobotContainer robotContainer) {
    super(robotContainer, robotContainer.getDrive().getPose());

    this.addCommands(
        new SequentialCommandGroup(
            followPath(Paths.OutToSSTrench), followPathAndSnowblow(Paths.TrenchToOut)));
  }
}
