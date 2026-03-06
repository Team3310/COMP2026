// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.Auton;

import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Container class for all PathPlanner paths and autonomous routines. Paths are loaded from
 * deploy/pathplanner/paths and autos from deploy/pathplanner/autos.
 */
public class Paths {
  // Individual paths (loaded from .path files)
  public static PathPlannerPath forward2m;

  public static PathPlannerPath depLoop;

  public static boolean loaded;

  public static void loadPaths(Alliance alliance) {
    loaded = false;
    if (alliance == Alliance.Blue) {

      forward2m = loadPath("forward2m");
      depLoop = loadPath("DepLoop");

      loaded = true;
    } else {

      forward2m = loadPath("forward2m"); // dont flip for this one
      depLoop = loadPath("DepLoop").flipPath();

      loaded = true;
    }
  }

  /** Load all paths from PathPlanner. */
  private static PathPlannerPath loadPath(String name) {
    try {
      return PathPlannerPath.fromPathFile(name);
    } catch (Exception e) {
      System.err.println("Failed to load path '" + name + "': " + e.getMessage());
    }
    return null;
  }
}
