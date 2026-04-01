package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class OutCollect extends AutonCommandBase {
  private static final Pose2d BLUE_START =
      new Pose2d(new Translation2d(4.445, 0.692), Rotation2d.fromDegrees(90.0));

  public OutCollect(RobotContainer robotContainer) {

    super(robotContainer, BLUE_START);

    this.addCommands(
        robotContainer.holdDefenceOutCommand().withTimeout(0.02),
        followPath(Paths.outCollect),
        followPath(Paths.outCollect2));
  }
}
