package frc.robot.Auton;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.Robot.OverrideState;
import frc.robot.RobotContainer;

/**
 * Container class for all autonomous commands. Automatically discovers and registers all public
 * Command methods for easy access.
 */
public class AutonCommandBase extends SequentialCommandGroup {
  private final RobotContainer robotContainer;
  private final Pose2d startingPose;

  /**
   * Creates a new AutonCommandBase.
   *
   * @param robotContainer The robot container
   * @param startingPose The starting pose of the robot
   */
  protected AutonCommandBase(RobotContainer robotContainer, Pose2d startingPose) {
    this.robotContainer = robotContainer;
    this.startingPose = startingPose;
  }

  /**
   * Returns the starting pose, or null if this auto should run from wherever the robot currently is
   * (no pose reset).
   */
  public Pose2d getStartingPose() {
    if (startingPose == null) return null;
    return new Pose2d(
        this.startingPose.getTranslation(),
        Robot.currentAlliance == Alliance.Red
            ? Rotation2d.fromDegrees(180)
            : Rotation2d.fromDegrees(0));
  }

  protected Command followPath(PathPlannerPath path) {
    return new FollowPathCommand(robotContainer.getDrive(), path);
  }

  protected Command followPathAndSnowblow(PathPlannerPath path) {
    return new ParallelCommandGroup(
        followPath(path), new InstantCommand(() -> Robot.overrideState = OverrideState.SNOWBLOW));
  }
}
