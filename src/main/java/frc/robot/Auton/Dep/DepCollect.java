package frc.robot.Auton.Dep;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.FieldConstants;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class DepCollect extends AutonCommandBase {

  public DepCollect(RobotContainer robotContainer) {
    super(
        robotContainer,
        new Pose2d(
            Robot.getEffectiveAlliance() == Alliance.Blue
                ? FieldConstants.StartingPosition.BLUEDEP.getTranslation().getX()
                : FieldConstants.StartingPosition.REDDEP.getTranslation().getX(),
            Robot.getEffectiveAlliance() == Alliance.Blue
                ? FieldConstants.StartingPosition.BLUEDEP.getTranslation().getY()
                : FieldConstants.StartingPosition.REDDEP.getTranslation().getY(),
            Rotation2d.fromDegrees(180.0)));

    this.addCommands(
        new SequentialCommandGroup(
            followPath(Paths.depCollect), new WaitCommand(3.5),
            followPath(Paths.depCollect2), new WaitCommand(3.5)));
  }
}
