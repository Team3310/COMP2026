package frc.robot.Auton;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.RobotContainer;

/**
 * Container class for all autonomous commands. Automatically discovers and registers all public
 * Command methods for easy access.
 */
public class AutonCommandBase extends SequentialCommandGroup {
  private final RobotContainer robotContainer;
  private final Rotation2d startingRotation;

  /**
   * Creates a new AutonCommandBase.
   *
   * @param robotContainer The robot container
   * @param startingRotation The starting rotation of the robot
   */
  protected AutonCommandBase(RobotContainer robotContainer, Rotation2d startingRotation) {
    this.robotContainer = robotContainer;
    this.startingRotation = startingRotation;
  }

  public Rotation2d getStartingRotation() {
    return this.startingRotation.plus(
        Robot.currentAlliance == Alliance.Red
            ? Rotation2d.fromDegrees(180)
            : Rotation2d.fromDegrees(0));
  }

  protected Command followPath(PathPlannerPath path) {
    return new FollowPathCommand(robotContainer.getDrive(), path);
  }
}
