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

  public static PathPlannerPath HubCycle;
  public static PathPlannerPath DepCycle;
  public static PathPlannerPath OutCycle;

  public static PathPlannerPath OutToOut;
  public static PathPlannerPath DepToDep;

  public static PathPlannerPath DepToSSTrench;
  public static PathPlannerPath OutToSSTrench;
  public static PathPlannerPath TrenchToDepo;
  public static PathPlannerPath TrenchToOut;

  public static PathPlannerPath DepToOut;
  public static PathPlannerPath OutToDep;

  public static PathPlannerPath HubToDep;
  public static PathPlannerPath HubToOut;

  public static PathPlannerPath DepSnowShoot;
  public static PathPlannerPath OutSnowShoot;

  public static PathPlannerPath snowblowOutpostBumpToTrench;
  public static PathPlannerPath snowblowOutpostTrenchToBump;
  public static PathPlannerPath outSnowblowMidShort;
  public static PathPlannerPath outSnowblowMidShort2;
  public static PathPlannerPath outCollectMidShort;
  public static PathPlannerPath outCollectMidShort2;

  public static PathPlannerPath depSnowblowMidShort;
  public static PathPlannerPath depCollect;
  public static PathPlannerPath depCollect2;

  public static PathPlannerPath outCollect;
  public static PathPlannerPath outCollect2;

  public static PathPlannerPath citrus1;
  public static PathPlannerPath citrus2;

  public static PathPlannerPath depCitrus1;
  public static PathPlannerPath depCitrus2;

  public static boolean loaded;

  /**
   * Load all paths from PathPlanner path files. Alliance flipping is handled automatially by
   * AutoBuilder.followPath() at runtime, so no manual flip needed here.
   */
  public static void loadPaths() {
    loaded = true;

    // forward2m = loadPath("forward2m");
    // DepCycle = loadPath("DepCycle");
    // OutCycle = loadPath("OutCycle");
    // HubCycle = loadPath("HubCycle");
    // DepToDep = loadPath("DepToDep");
    // OutToOut = loadPath("OutToOut");
    // DepToSSTrench = loadPath("DepToSSTrench");
    // OutToSSTrench = loadPath("OutToSSTrench");
    // TrenchToDepo = loadPath("TrenchToDepo");
    // DepToOut = loadPath("DepToOut");
    // OutToDep = loadPath("OutToDep");
    // HubToDep = loadPath("HubToDep");
    // HubToOut = loadPath("HubToOut");
    // DepSnowShoot = loadPath("DepSnowShoot");
    // OutSnowShoot = loadPath("OutSnowShoot");
    // TrenchToOut = loadPath("TrenchToOut");

    // depSnowblowMidShort = loadPath("DepSnowblowMidShort");
    depCollect = loadPath("DepCollect");
    depCollect2 = loadPath("DepCollect2");

    outCollect = loadPath("OutCollect");
    outCollect2 = loadPath("OutCollect2");

    // snowblowOutpostBumpToTrench = loadPath("snowblowOutpostBumpToTrench");
    // snowblowOutpostTrenchToBump = loadPath("snowblowOutpostTrenchToBump");
    // outSnowblowMidShort = loadPath("OutSnowblowMidShort");
    // outSnowblowMidShort2 = loadPath("OutSnowblowMidShort2");
    // outCollectMidShort = loadPath("OutCollectMidShort");
    outCollectMidShort2 = loadPath("OutCollectMidShort2");

    citrus1 = loadPath("Citrus1");
    citrus2 = loadPath("Citrus2");

    depCitrus1 = loadPath("DepCitrus1");
    depCitrus2 = loadPath("DepCitrus2");
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
