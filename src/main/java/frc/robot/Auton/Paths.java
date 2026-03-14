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

  public static PathPlannerPath outtoout;
  public static PathPlannerPath deptodep;

  public static PathPlannerPath deptosstrenchcycle;
  public static PathPlannerPath outtosstrenchcycle;

  public static PathPlannerPath deptoout;
  public static PathPlannerPath outtodep;

  public static PathPlannerPath hubtodep;
  public static PathPlannerPath hubtoout;

  public static boolean loaded;

  public static void loadPaths(Alliance alliance) {
    loaded = true;
    if (alliance == Alliance.Blue) {

      forward2m = loadPathForAlliance("forward2m", false);

      depCycle = loadPathForAlliance("DepCycle", false);
      outCycle = loadPathForAlliance("OutCycle", false);
      hubCycle = loadPathForAlliance("HubCycle", false);

      deptodep = loadPathForAlliance("DepToDep", false);
      outtoout = loadPathForAlliance("OutToOut", false);

      deptosstrenchcycle = loadPathForAlliance("DepToSSTrenchCycle", false);
      outtosstrenchcycle = loadPathForAlliance("OutToSSTrenchCycle", false);

      deptoout = loadPathForAlliance("DepToOut", false);
      outtodep = loadPathForAlliance("OutToDep", false);

      hubtodep = loadPathForAlliance("HubToDep", false);
      hubtoout = loadPathForAlliance("HubToOut", false);
    } else {

      forward2m = loadPathForAlliance("forward2m", false); // dont flip for this one

      depCycle = loadPathForAlliance("DepCycle", true);
      outCycle = loadPathForAlliance("OutCycle", true);
      hubCycle = loadPathForAlliance("HubCycle", true);

      deptodep = loadPathForAlliance("DepToDep", true);
      outtoout = loadPathForAlliance("OutToOut", true);

      deptosstrenchcycle = loadPathForAlliance("DepToSSTrenchCycle", true);
      outtosstrenchcycle = loadPathForAlliance("OutToSSTrenchCycle", true);

      deptoout = loadPathForAlliance("DepToOut", true);
      outtodep = loadPathForAlliance("OutToDep", true);

      hubtodep = loadPathForAlliance("HubToDep", true);
      hubtoout = loadPathForAlliance("HubToOut", true);
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
