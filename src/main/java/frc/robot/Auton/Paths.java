// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.Auton;

import com.pathplanner.lib.path.PathPlannerPath;

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
  public static PathPlannerPath trenchToDepo;

  public static PathPlannerPath depToOut;
  public static PathPlannerPath outToDep;

  public static PathPlannerPath hubToDep;
  public static PathPlannerPath hubToOut;

  public static boolean loaded;

  /**
   * Load all paths from PathPlanner path files. Alliance flipping is handled automatically by
   * AutoBuilder.followPath() at runtime, so no manual flip needed here.
   */
  public static void loadPaths() {
    loaded = true;

    forward2m = loadPath("forward2m");
    depCycle = loadPath("DepCycle");
    outCycle = loadPath("OutCycle");
    hubCycle = loadPath("HubCycle");
    depToDep = loadPath("DepToDep");
    outToOut = loadPath("OutToOut");
    depToSSTrench = loadPath("DepToSSTrench");
    outToSSTrench = loadPath("OutToSSTrench");
    trenchToDepo = loadPath("TrenchToDepo");
    depToOut = loadPath("DepToOut");
    outToDep = loadPath("OutToDep");
    hubToDep = loadPath("HubToDep");
    hubToOut = loadPath("HubToOut");
  }

  /** Load a single path from a PathPlanner .path file. */
  private static PathPlannerPath loadPath(String name) {
    try {
      return PathPlannerPath.fromPathFile(name);
    } catch (Exception e) {
      System.err.println("Failed to load path '" + name + "': " + e.getMessage());
      loaded = false;
    }
    return null;
  }
}
