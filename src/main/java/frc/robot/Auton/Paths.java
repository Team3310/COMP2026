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

  public static PathPlannerPath hubCycle;
  public static PathPlannerPath depCycle;
  public static PathPlannerPath outCycle;

  public static PathPlannerPath outToOut;
  public static PathPlannerPath depToDep;

  public static PathPlannerPath depToSSTrench;
  public static PathPlannerPath outToSSTrench;

  public static PathPlannerPath depToOut;
  public static PathPlannerPath outToDep;

  public static PathPlannerPath hubToDep;
  public static PathPlannerPath hubToOut;

  public static boolean loaded;

  public static void loadPaths(Alliance alliance) {
    loaded = true;
    if (alliance == Alliance.Blue) {

      forward2m = loadPathForAlliance("forward2m", false);

      depCycle = loadPathForAlliance("DepCycle", false);
      outCycle = loadPathForAlliance("OutCycle", false);
      hubCycle = loadPathForAlliance("HubCycle", false);

      depToDep = loadPathForAlliance("DepToDep", false);
      outToOut = loadPathForAlliance("OutToOut", false);

      depToSSTrench = loadPathForAlliance("DepToSSTrench", false);
      outToSSTrench = loadPathForAlliance("OutToSSTrench", false);
      depToOut = loadPathForAlliance("DepToOut", false);
      outToDep = loadPathForAlliance("OutToDep", false);

      hubToDep = loadPathForAlliance("HubToDep", false);
      hubToOut = loadPathForAlliance("HubToOut", false);
    } else {

      forward2m = loadPathForAlliance("forward2m", false); // dont flip for this one

      depCycle = loadPathForAlliance("DepCycle", true);
      outCycle = loadPathForAlliance("OutCycle", true);
      hubCycle = loadPathForAlliance("HubCycle", true);

      depToDep = loadPathForAlliance("DepToDep", true);
      outToOut = loadPathForAlliance("OutToOut", true);

      depToSSTrench = loadPathForAlliance("DepToSSTrench", true);
      outToSSTrench = loadPathForAlliance("OutToSSTrench", true);
      depToOut = loadPathForAlliance("DepToOut", true);
      outToDep = loadPathForAlliance("OutToDep", true);

      hubToDep = loadPathForAlliance("HubToDep", true);
      hubToOut = loadPathForAlliance("HubToOut", true);
    }
  }

  private static PathPlannerPath loadPathForAlliance(String name, boolean flip) {
    PathPlannerPath path = loadPath(name);
    if (path == null) {
      loaded = false;
      return null;
    }
    return flip ? path.flipPath() : path;
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
