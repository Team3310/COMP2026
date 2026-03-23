package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class Citrus1 extends AutonCommandBase {

  public Citrus1(RobotContainer robotContainer) {
    super(
        robotContainer,
        new Pose2d(new Translation2d(4.433125, 0.7584305555555546), Rotation2d.fromDegrees(180.0)));

    this.addCommands(followPath(Paths.citrus1));
  }
}
