package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class OutCitrus extends AutonCommandBase {

  private static final Pose2d BLUE_START =
      new Pose2d(new Translation2d(4.433125, 0.7584305555555546), Rotation2d.fromDegrees(90.0));

  public OutCitrus(RobotContainer robotContainer) {
    super(robotContainer, BLUE_START);

    this.addCommands(
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                followPathAndCollectThenShoot(Paths.outCitrus1),
                followPathAndCollectThenShoot(Paths.outCitrus2)),
            flywheelsOn()));
  }
}
