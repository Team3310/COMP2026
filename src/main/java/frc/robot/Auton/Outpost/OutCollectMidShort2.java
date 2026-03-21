package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.lib.util.FieldConstants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class OutCollectMidShort2 extends AutonCommandBase {

  public OutCollectMidShort2(RobotContainer robotContainer) {
            super(robotContainer,
                new Pose2d(
                        Robot.getEffectiveAlliance() == Alliance.Blue
                                ? FieldConstants.StartingPosition.BLUEOUT.getTranslation().getX()
                                : FieldConstants.StartingPosition.REDOUT.getTranslation().getX(),
                        Robot.getEffectiveAlliance() == Alliance.Blue
                                ? FieldConstants.StartingPosition.BLUEOUT.getTranslation().getY()
                                : FieldConstants.StartingPosition.REDOUT.getTranslation().getY(),
                        Rotation2d.fromDegrees(180.0)));

    this.addCommands(
        new SequentialCommandGroup(
            followPath(Paths.outCollectMidShort),
            new WaitCommand(3.5),
            followPath(Paths.outCollectMidShort),
            new WaitCommand(3.5)));
  }
}
