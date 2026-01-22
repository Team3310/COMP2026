// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/**
 * General command that follows a PathPlanner path. Can be used to follow any path by passing in the
 * path object.
 */
public class FollowPathCommand extends Command {
  private final PathPlannerPath path;
  private Command pathCommand;

  /**
   * Creates a new FollowPathCommand.
   *
   * @param drive The drive subsystem (required for command scheduler)
   * @param path The PathPlanner path to follow
   */
  public FollowPathCommand(Drive drive, PathPlannerPath path) {
    this.path = path;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Create the path following command when this command starts
    if (path != null) {
      pathCommand = AutoBuilder.followPath(path);
      pathCommand.schedule();
    }
  }

  @Override
  public void execute() {
    // Path following handled by pathCommand
  }

  @Override
  public void end(boolean interrupted) {
    if (pathCommand != null) {
      pathCommand.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return pathCommand == null || pathCommand.isFinished();
  }
}
