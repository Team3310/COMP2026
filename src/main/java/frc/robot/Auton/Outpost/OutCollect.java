package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class OutCollect extends AutonCommandBase {
  private static final Pose2d BLUE_START =
      new Pose2d(new Translation2d(4.445, 0.692), Rotation2d.fromDegrees(90.0));

  public OutCollect(RobotContainer robotContainer) {

    super(robotContainer, BLUE_START);

    this.addCommands(
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                followPathDeployImmediatelyAndCollectThenShoot(Paths.outCollect),
                followPathAndCollectThenShoot(Paths.outCollect2)),
            flywheelsOn()));
  }
}
