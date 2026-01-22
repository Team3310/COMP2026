// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.Auton;

import com.pathplanner.lib.path.PathPlannerPath;

/**
 * Container class for all PathPlanner paths and autonomous routines. Paths are loaded from
 * deploy/pathplanner/paths and autos from deploy/pathplanner/autos.
 */
public class paths {
  // Individual paths (loaded from .path files)
  public PathPlannerPath forward2m;

  /** Creates a new Paths container. */
  public paths() {
    loadPaths();
  }

  /** Load all paths from PathPlanner. */
  private void loadPaths() {
    try {
      forward2m = PathPlannerPath.fromPathFile("forward2m");
    } catch (Exception e) {
      System.err.println("Failed to load path 'forward2m': " + e.getMessage());
    }
  }
}
