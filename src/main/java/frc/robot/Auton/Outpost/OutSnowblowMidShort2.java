package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class OutSnowblowMidShort2 extends AutonCommandBase {

  public OutSnowblowMidShort2(RobotContainer robotContainer) {
    super(robotContainer, Rotation2d.fromDegrees(180.0));
    this.addCommands(
        new SequentialCommandGroup(
            followPath(Paths.outSnowblowMidShort2),
            new WaitCommand(3.0),
            followPath(Paths.outSnowblowMidShort2),
            new WaitCommand(3.0)));
  }
}
