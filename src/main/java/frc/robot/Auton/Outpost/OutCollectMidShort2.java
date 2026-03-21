package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class OutCollectMidShort2 extends AutonCommandBase {

  public OutCollectMidShort2(RobotContainer robotContainer) {
    super(robotContainer, Rotation2d.fromDegrees(180.0)); // CHANGE TO 180!!!!

    this.addCommands(
        new SequentialCommandGroup(
            followPath(Paths.outCollectMidShort),
            new WaitCommand(4.0),
            followPath(Paths.outCollectMidShort),
            new WaitCommand(4.0)));
  }
}
