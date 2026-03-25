package frc.robot.Auton.Dep;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class DepCitrus extends AutonCommandBase {

  private static final Pose2d BLUE_START =
      new Pose2d(new Translation2d(4.433125, 7.3195694444444454), Rotation2d.fromDegrees(-90.0));

  public DepCitrus(RobotContainer robotContainer) {
    super(robotContainer, BLUE_START);

    this.addCommands(
        new SequentialCommandGroup(
            followPath(Paths.depCitrus1), new WaitCommand(4.0), followPath(Paths.depCitrus2)));
  }
}
