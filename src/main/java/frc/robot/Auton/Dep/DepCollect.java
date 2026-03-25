package frc.robot.Auton.Dep;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class DepCollect extends AutonCommandBase {
  private static final Pose2d BLUE_START =
      new Pose2d(new Translation2d(4.445, 7.386), Rotation2d.fromDegrees(-90.0));

  public DepCollect(RobotContainer robotContainer) {

    super(robotContainer, BLUE_START);

    // Flywheels run for the entire auto, continuously tracking TurretAimManager
    // targets. The SequentialCommandGroup (paths + shoot cycles) is the deadline
    // so flywheels are cancelled when the last path/shoot finishes.
    this.addCommands(
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                followPathAndCollectThenShoot(Paths.depCollect),
                followPathAndCollectThenShoot(Paths.depCollect2)),
            flywheelsOn()));
  }
}
