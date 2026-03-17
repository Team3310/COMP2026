package frc.robot.Auton.Dep;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.lib.util.FieldConstants;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.RobotContainer;

public class DepToDep extends AutonCommandBase {
  public DepToDep(RobotContainer robotContainer) {
    super(
        robotContainer,
        new Pose2d(
            FieldConstants.StartingPosition.BLUEDEP.getTranslation(), Rotation2d.fromDegrees(180)));

    this.addCommands(followPathAndSnowblow(Paths.DepToDep));
  }
}
