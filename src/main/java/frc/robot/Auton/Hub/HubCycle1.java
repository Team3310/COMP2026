package frc.robot.Auton.Hub;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.lib.util.FieldConstants;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class HubCycle1 extends AutonCommandBase {
  public HubCycle1(RobotContainer robotContainer) {
    super(
        robotContainer,
        new Pose2d(
            Robot.getEffectiveAlliance() == Alliance.Blue
                ? FieldConstants.StartingPosition.BLUEHUB.getTranslation().getX()
                : FieldConstants.StartingPosition.REDHUB.getTranslation().getX(),
            Robot.getEffectiveAlliance() == Alliance.Blue
                ? FieldConstants.StartingPosition.BLUEHUB.getTranslation().getY()
                : FieldConstants.StartingPosition.REDHUB.getTranslation().getY(),
            Rotation2d.fromDegrees(180.0)));

    this.addCommands(followPathAndSnowblow(Paths.HubCycle));
  }
}
