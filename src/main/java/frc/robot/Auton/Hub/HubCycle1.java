package frc.robot.Auton.Hub;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.FieldConstants;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class HubCycle1 extends AutonCommandBase {
  public HubCycle1(RobotContainer robotContainer) {
    super(
        robotContainer,
        new Pose2d(
            FieldConstants.StartingPosition.BLUEHUB.getTranslation(),
            Rotation2d.fromDegrees(180.0)));

    this.addCommands(followPathAndSnowblow(Paths.HubCycle));
  }
}
