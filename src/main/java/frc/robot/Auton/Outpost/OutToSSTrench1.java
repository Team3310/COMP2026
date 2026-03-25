package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.util.FieldConstants;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class OutToSSTrench1 extends AutonCommandBase {
  public OutToSSTrench1(RobotContainer robotContainer) {
    super(
        robotContainer,
        new Pose2d(
            Robot.getEffectiveAlliance() == Alliance.Blue
                ? FieldConstants.StartingPosition.BLUEOUTHOME.getTranslation().getX()
                : FieldConstants.StartingPosition.REDOUTHOME.getTranslation().getX(),
            Robot.getEffectiveAlliance() == Alliance.Blue
                ? FieldConstants.StartingPosition.BLUEOUTHOME.getTranslation().getY()
                : FieldConstants.StartingPosition.REDOUTHOME.getTranslation().getY(),
            Rotation2d.fromDegrees(180.0)));

    this.addCommands(
        new SequentialCommandGroup(
            followPath(Paths.OutToSSTrench), followPath(Paths.snowblowOutpostBumpToTrench)));
  }
}
