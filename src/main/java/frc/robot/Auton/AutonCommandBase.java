package frc.robot.Auton;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.util.FieldConstants.Zone;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Robot.OverrideState;
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
   * @param startingRotation The starting rotation of the robot, or null for no pose reset
   */
  protected AutonCommandBase(RobotContainer robotContainer, Rotation2d startingRotation) {
    this.robotContainer = robotContainer;
    this.startingRotation = startingRotation;
  }

  /**
   * Returns the starting pose (current drive translation + the provided starting rotation adjusted
   * for alliance), or null if this auto should run from wherever the robot currently is (no pose
   * reset).
   */
  public Rotation2d getStartingRotation() {
    if (startingRotation == null) return null;
    return startingRotation.plus(
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

  /**
   * Returns a command that waits until the robot is in its home alliance zone AND at least one
   * flywheel is within {@link Constants.ScorerConstants#kFlywheelRPMTolerance} of its target. A
   * single flywheel being ready is sufficient — waiting for both risks blocking forever if one
   * motor has an issue.
   *
   * <p>Falls back to a 3-second timeout so the auto never hangs indefinitely.
   */
  protected Command waitForFlywheelsAtSpeed() {
    return new WaitUntilCommand(
            () -> {
              // Confirm the robot is in its home alliance zone before shooting.
              boolean inHomeZone =
                  Robot.currentAlliance == Alliance.Blue
                      ? Robot.currentZone == Zone.BLUE
                      : Robot.currentZone == Zone.RED;

              double tolRPM = Constants.ScorerConstants.kFlywheelRPMTolerance;
              double leftTarget = robotContainer.getTurretAimManager().getLeftFlywheelRPM();
              double rightTarget = robotContainer.getTurretAimManager().getRightFlywheelRPM();
              double leftErr =
                  Math.abs(robotContainer.getFlywheelLeft().getCurrentVelocity() - leftTarget);
              double rightErr =
                  Math.abs(robotContainer.getFlywheelRight().getCurrentVelocity() - rightTarget);
              return inHomeZone && (leftErr < tolRPM || rightErr < tolRPM);
            })
        .withTimeout(3.0)
        .withName("WaitForFlywheelsAtSpeed");
  }
}
