package frc.robot.Auton.Outpost;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldConstants;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class OutCycle1 extends AutonCommandBase {
  public OutCycle1(RobotContainer robotContainer) {
    super(
        robotContainer,
        new Pose2d(
            Robot.getEffectiveAlliance() == Alliance.Blue
                ? FieldConstants.StartingPosition.BLUEOUT.getTranslation().getX()
                : FieldConstants.StartingPosition.REDOUT.getTranslation().getX(),
            Robot.getEffectiveAlliance() == Alliance.Blue
                ? FieldConstants.StartingPosition.BLUEOUT.getTranslation().getY()
                : FieldConstants.StartingPosition.REDOUT.getTranslation().getY(),
            Rotation2d.fromDegrees(180.0)));

    this.addCommands(followPathAndSnowblow(Paths.OutCycle));
  }
}
