package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class OutSnowblowMidShort extends AutonCommandBase {

  public OutSnowblowMidShort(RobotContainer robotContainer) {
    super(robotContainer, Rotation2d.fromDegrees(180.0)); // CHANGE TO 180!!!!

    this.addCommands(
        new SequentialCommandGroup(
            followPath(Paths.outSnowblowMidShort), new WaitCommand(2.0)
            // followPath(Paths.snowblowMidShort)
            ));
  }
}
