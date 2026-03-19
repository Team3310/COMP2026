package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class OutToSSTrench1 extends AutonCommandBase {
  public OutToSSTrench1(RobotContainer robotContainer) {
    super(robotContainer, Rotation2d.fromDegrees(90.0));

    this.addCommands(
        new SequentialCommandGroup(
            followPath(Paths.OutToSSTrench), followPath(Paths.snowblowOutpostBumpToTrench)));
  }
}
