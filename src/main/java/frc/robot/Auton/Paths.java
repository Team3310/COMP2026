// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.Auton;

import java.nio.file.Path;

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
    loaded = false;
    if (alliance == Alliance.Blue) {

      forward2m = loadPath("forward2m");

      depCycle = loadPath("DepLoop");
      outCycle = loadPath("OutLoop");
      hubCycle = loadPath("HubLoop");
      
      deptodep = loadPath("DepToDep");
      outtoout = loadPath("OutToOut");

      deptosstrenchcycle = loadPath("DepToSSTrenchCycle");
      outtosstrenchcycle = loadPath("OutToSSTrenchCycle");

      deptoout = loadPath("DepToOut");
      outtodep = loadPath("OutToDep");

      hubtodep = loadPath("HubToDep");
      hubtoout = loadPath("HubToOut");

      loaded = true;
    } else {

      forward2m = loadPath("forward2m"); // dont flip for this one

      depCycle = loadPath("DepLoop").flipPath();
      outCycle = loadPath("OutLoop").flipPath();
      hubCycle = loadPath("HubLoop").flipPath();
      
      deptodep = loadPath("DepToDep").flipPath();
      outtoout = loadPath("OutToOut").flipPath();

      deptosstrenchcycle = loadPath("DepToSSTrenchCycle").flipPath();
      outtosstrenchcycle = loadPath("OutToSSTrenchCycle").flipPath();

      deptoout = loadPath("DepToOut").flipPath();
      outtodep = loadPath("OutToDep").flipPath();
      
      hubtodep = loadPath("HubToDep").flipPath();
      hubtoout = loadPath("HubToOut").flipPath();
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
