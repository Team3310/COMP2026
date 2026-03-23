package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class CitrusAuton extends AutonCommandBase {

  public CitrusAuton(RobotContainer robotContainer) {
    super(
        robotContainer,
        new Pose2d(new Translation2d(4.433125, 0.7584305555555546), Rotation2d.fromDegrees(180.0)));

    this.addCommands(
        new SequentialCommandGroup(
            followPath(Paths.citrus1), new WaitCommand(4.0), followPath(Paths.citrus2)));
  }
}
