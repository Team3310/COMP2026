// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.util.FieldConstants;
import frc.lib.util.FieldConstants.Zone;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.subsystems.Lights;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {

  // enums
  public static enum BotState {
    SNOWBLOW,
    COLLECT,
    DEFENCEIN,
    DEFENCEOUT,
    DEPLOY,
    RETRACT,
    TRENCH,
    PIT
  }

  public static enum OverrideState {
    OFF,
    ON,
    COLLECT,
    DEFENCEIN,
    DEFENCEOUT,
    SNOWBLOW
  }

  // Variables

  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public static boolean inPit = false;

  public static BotState currentState = BotState.PIT;
  public static OverrideState overrideState = OverrideState.OFF;
  public static boolean deploying = false;
  public static boolean retracting = false;
  public static boolean activeHub = true;
  public static boolean hubOverride = false; // true = manually forced OFF by SmartDashboard button
  public static Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  public static FieldConstants.Zone currentZone = FieldConstants.Zone.BLUE;
  public static Lights.LightMode currentLightMode = Lights.LightMode.OFF;
  private BotState lastAppliedState = null;

  public Robot() {
    // Record metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });

    // Set up data receivers & replay source
    switch (Constants.currentMode) {
      case REAL:
        // Running on a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case SIM:
        // Running a physics simulator, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        // Replaying a log, set up replay source
        setUseTiming(false); // Run as fast as possible
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    Logger.start();

    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our autonomous chooser on the dashboard.
    robotContainer = RobotContainer.getInstance();
  }

  /** This function is called periodically during all modes. */
  @java.lang.Override
  public void robotPeriodic() {
    // Optionally switch the thread to high priority to improve loop
    // timing (see the template project documentation for details)
    // Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled commands, running already-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodic() methods.
    // This must be called from the robot's periodic block in order for anything in
    // the Command-based framework to work.
    CommandScheduler.getInstance().run();

    double robotX = robotContainer.getDrive().getPose().getX();
    double robotY = robotContainer.getDrive().getPose().getY();
    double robotOrient = robotContainer.getDrive().getPose().getRotation().getDegrees();
    Zone currentZone =
        robotX < Zone.BLUE.getX() ? Zone.BLUE : robotX < Zone.MID.getX() ? Zone.MID : Zone.RED;

    updateZone();

    // Update activeHub based on match time / override flag
    updateHub();

    // This basically says if we are not deploying or retracting, then we can change
    // states.
    // If we are deploying or retracting, we want to stay in deploy or retract until
    // we are done.
    if (inPit) {
      currentState = BotState.PIT;
    } else if (!deploying && !retracting) {
      switch (overrideState) {
        case OFF: // nothing runs
          currentState = BotState.PIT;
          break;
        case COLLECT:
          currentState = BotState.COLLECT;
          break;
        case DEFENCEIN:
          currentState = BotState.DEFENCEIN;
          break;
        case DEFENCEOUT:
          currentState = BotState.DEFENCEOUT;
          break;
        case SNOWBLOW:
          currentState = BotState.SNOWBLOW;
          break;
        case ON:
        default:
          // Automated State Machine
          if (currentZone == Zone.BLUE && currentAlliance == Alliance.Blue && !(activeHub)) {
            currentState = BotState.COLLECT;
          } else if (currentZone == Zone.RED && currentAlliance == Alliance.Red && !(activeHub)) {
            currentState = BotState.COLLECT;
          } else if (isInTrenchZone()) {
            currentState = BotState.TRENCH;
          } else {
            currentState = BotState.SNOWBLOW;
          }
      }
    }

    if (currentState != lastAppliedState) {
      onStateEntered(currentState);
      lastAppliedState = currentState;
    }

    if (shouldTrack(currentState)) {
      track();
    }

    // logging
    SmartDashboard.putBoolean("inPit", inPit);
    SmartDashboard.putString("currentState", "" + currentState);
    SmartDashboard.putString("overrideState", "" + overrideState);
    SmartDashboard.putString("robotX", "" + robotX);
    SmartDashboard.putString("robotY", "" + robotY);
    SmartDashboard.putString("robotOrient", "" + robotOrient);
    SmartDashboard.putString("currentZone", "" + currentZone);
    SmartDashboard.putBoolean("activeHub", activeHub);
    SmartDashboard.putBoolean("hubOverride", hubOverride);
    SmartDashboard.putNumber("matchTime", DriverStation.getMatchTime());

    // Read speed tuning overrides from SmartDashboard into Constants (updated each
    // loop)
    Constants.AgitatorConstants.kFloorRollerSnowblowRPM =
        SmartDashboard.getNumber(
            "SpeedTune/FloorForwardRPM", Constants.AgitatorConstants.kFloorRollerSnowblowRPM);
    Constants.AgitatorConstants.kFloorRollerReverseRPM =
        SmartDashboard.getNumber(
            "SpeedTune/FloorReverseRPM", Constants.AgitatorConstants.kFloorRollerReverseRPM);
    Constants.IntakeConstants.kIntakeVelocityRPM =
        SmartDashboard.getNumber(
            "SpeedTune/IntakeForwardRPM", Constants.IntakeConstants.kIntakeVelocityRPM);
    Constants.IntakeConstants.kOuttakeVelocityRPM =
        SmartDashboard.getNumber(
            "SpeedTune/IntakeReverseRPM", Constants.IntakeConstants.kOuttakeVelocityRPM);
    Constants.AgitatorConstants.kVerticalFeedIntakeRPM =
        SmartDashboard.getNumber(
            "SpeedTune/VertFeedForwardRPM", Constants.AgitatorConstants.kVerticalFeedIntakeRPM);
    Constants.AgitatorConstants.kVerticalFeedOuttakeRPM =
        SmartDashboard.getNumber(
            "SpeedTune/VertFeedReverseRPM", Constants.AgitatorConstants.kVerticalFeedOuttakeRPM);
    Constants.ScorerConstants.kShootRPM =
        SmartDashboard.getNumber(
            "SpeedTune/FlywheelForwardRPM", Constants.ScorerConstants.kShootRPM);
    Constants.ScorerConstants.kReverseShootRPM =
        SmartDashboard.getNumber(
            "SpeedTune/FlywheelReverseRPM", Constants.ScorerConstants.kReverseShootRPM);
    Constants.ScorerConstants.kHoodMaxDegrees =
        SmartDashboard.getNumber(
            "SpeedTune/HoodMaxDegrees", Constants.ScorerConstants.kHoodMaxDegrees);
    Constants.ScorerConstants.kTurretMaxPositionUnits =
        SmartDashboard.getNumber(
            "SpeedTune/TurretMaxDegrees", Constants.ScorerConstants.kTurretMaxPositionUnits);
    Constants.ScorerConstants.kTofSeconds =
        SmartDashboard.getNumber(
            "SpeedTune/TofSeconds", Constants.ScorerConstants.kTofSeconds);

    // Per-subsystem supply current (amps)
    double iFloorLeft = robotContainer.getAgitatorLeft().getSupplyCurrentAmps();
    double iFloorRight = robotContainer.getAgitatorRight().getSupplyCurrentAmps();
    double iVertLeft = robotContainer.getVerticalFeedLeft().getSupplyCurrentAmps();
    double iVertRight = robotContainer.getVerticalFeedRight().getSupplyCurrentAmps();
    double iIntake = robotContainer.getIntakeRollers().getSupplyCurrentAmps();
    double iPivot = robotContainer.getIntakePivot().getSupplyCurrentAmps();
    double iFlywheelL = robotContainer.getFlywheelLeft().getSupplyCurrentAmps();
    double iFlywheelR = robotContainer.getFlywheelRight().getSupplyCurrentAmps();
    double iHoodL = robotContainer.getHoodLeft().getSupplyCurrentAmps();
    double iHoodR = robotContainer.getHoodRight().getSupplyCurrentAmps();
    double iTurretL = robotContainer.getTurretLeft().getSupplyCurrentAmps();
    double iTurretR = robotContainer.getTurretRight().getSupplyCurrentAmps();

    SmartDashboard.putNumber("Current/FloorLeft_A", iFloorLeft);
    SmartDashboard.putNumber("Current/FloorRight_A", iFloorRight);
    SmartDashboard.putNumber("Current/VertFeedLeft_A", iVertLeft);
    SmartDashboard.putNumber("Current/VertFeedRight_A", iVertRight);
    SmartDashboard.putNumber("Current/IntakeRoller_A", iIntake);
    SmartDashboard.putNumber("Current/IntakePivot_A", iPivot);
    SmartDashboard.putNumber("Current/FlywheelLeft_A", iFlywheelL);
    SmartDashboard.putNumber("Current/FlywheelRight_A", iFlywheelR);
    SmartDashboard.putNumber("Current/HoodLeft_A", iHoodL);
    SmartDashboard.putNumber("Current/HoodRight_A", iHoodR);
    SmartDashboard.putNumber("Current/TurretLeft_A", iTurretL);
    SmartDashboard.putNumber("Current/TurretRight_A", iTurretR);
    SmartDashboard.putNumber(
        "Current/TotalMotors_A",
        iFloorLeft
            + iFloorRight
            + iVertLeft
            + iVertRight
            + iIntake
            + iPivot
            + iFlywheelL
            + iFlywheelR
            + iHoodL
            + iHoodR
            + iTurretL
            + iTurretR);
    SmartDashboard.putNumber(
        "Current/Drive_FL_A", robotContainer.getDrive().getModuleDriveCurrentAmps(0));
    SmartDashboard.putNumber(
        "Current/Drive_FR_A", robotContainer.getDrive().getModuleDriveCurrentAmps(1));
    SmartDashboard.putNumber(
        "Current/Drive_BL_A", robotContainer.getDrive().getModuleDriveCurrentAmps(2));
    SmartDashboard.putNumber(
        "Current/Drive_BR_A", robotContainer.getDrive().getModuleDriveCurrentAmps(3));
    SmartDashboard.putNumber(
        "Current/Turn_FL_A", robotContainer.getDrive().getModuleTurnCurrentAmps(0));
    SmartDashboard.putNumber(
        "Current/Turn_FR_A", robotContainer.getDrive().getModuleTurnCurrentAmps(1));
    SmartDashboard.putNumber(
        "Current/Turn_BL_A", robotContainer.getDrive().getModuleTurnCurrentAmps(2));
    SmartDashboard.putNumber(
        "Current/Turn_BR_A", robotContainer.getDrive().getModuleTurnCurrentAmps(3));
    SmartDashboard.putNumber(
        "Current/TotalDrivetrain_A", robotContainer.getDrive().getTotalDriveTrainCurrentAmps());
    SmartDashboard.putNumber(
        "Current/TotalRobot_A",
        iFloorLeft
            + iFloorRight
            + iVertLeft
            + iVertRight
            + iIntake
            + iPivot
            + iFlywheelL
            + iFlywheelR
            + iHoodL
            + iHoodR
            + iTurretL
            + iTurretR
            + robotContainer.getDrive().getTotalDriveTrainCurrentAmps());

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    Paths.loadPaths(currentAlliance);

    autonomousCommand = robotContainer.getAutonomousCommand();

    robotContainer.getDrive().setPose(((AutonCommandBase) autonomousCommand).getStartingPose());

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  // #region util methods
  public void updateZone() {
    double robotX = robotContainer.getDrive().getPose().getX();

    if (robotX < FieldConstants.Zone.BLUE.getX()) {
      currentZone = Zone.BLUE;
    } else if (robotX < FieldConstants.Zone.BLUETRENCH.getX()) {
      currentZone = Zone.BLUETRENCH;
    } else if (robotX < FieldConstants.Zone.MID.getX()) {
      currentZone = Zone.MID;
    } else if (robotX < FieldConstants.Zone.REDTRENCH.getX()) {
      currentZone = Zone.REDTRENCH;
    } else {
      currentZone = Zone.RED;
    }
  }

  public void updateHub() {
    // If manually overridden OFF via SmartDashboard button, skip automatic
    // calculation
    if (hubOverride) {
      activeHub = false;
      return;
    }

    // Hub is always enabled in autonomous.
    if (DriverStation.isAutonomousEnabled()) {
      activeHub = true;
      return;
    }
    // At this point, if we're not teleop enabled, there is no hub.
    if (!DriverStation.isTeleopEnabled()) {
      return;
    }

    // We're teleop enabled, compute.
    double matchTime = DriverStation.getMatchTime();
    String gameData = DriverStation.getGameSpecificMessage();
    // If we have no game data, we cannot compute, assume hub is active, as its
    // likely early in
    // teleop.
    if (matchTime < 0 && (gameData == null || gameData.isEmpty())) {
      activeHub = true;
      return;
    }
    boolean redInactiveFirst = false;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveFirst = true;
      case 'B' -> redInactiveFirst = false;
      default -> {
        // If we have invalid game data, assume hub is active.
        activeHub = true;
        return;
      }
    }

    // Shift was is active for blue if red won auto, or red if blue won auto.
    boolean shift1Active =
        switch (currentAlliance) {
          case Red -> !redInactiveFirst;
          case Blue -> redInactiveFirst;
        };

    if (matchTime > 130) {
      // Transition shift, hub is active.
      activeHub = true;
      return;
    } else if (matchTime > 105) {
      // Shift 1
      activeHub = shift1Active;
      return;
    } else if (matchTime > 80) {
      // Shift 2
      activeHub = !shift1Active;
      return;
    } else if (matchTime > 55) {
      // Shift 3
      activeHub = shift1Active;
      return;
    } else if (matchTime > 30) {
      // Shift 4
      activeHub = !shift1Active;
      return;
    } else {
      // End game, hub always active.
      activeHub = true;
      return;
    }
  }

  private boolean isInTrenchZone() {
    if (currentZone == Zone.BLUETRENCH || currentZone == Zone.REDTRENCH) {
      return true;
    }
    return false;
  }

  private void track() {
    CommandScheduler.getInstance()
        .schedule(
            robotContainer
                .getTurretLeft()
                .setDegreesCommand(robotContainer.getTurretAimManager().getLeftTurretAngleDeg()));
    CommandScheduler.getInstance()
        .schedule(
            robotContainer
                .getTurretRight()
                .setDegreesCommand(robotContainer.getTurretAimManager().getRightTurretAngleDeg()));
    // CommandScheduler.getInstance()
    //     .schedule(
    //         robotContainer
    //             .getHoodLeft()
    //             .setDegreesCommand(robotContainer.getTurretAimManager().getLeftHoodAngleDeg()));
    // CommandScheduler.getInstance()
    //     .schedule(
    //         robotContainer
    //             .getHoodRight()
    //             .setDegreesCommand(robotContainer.getTurretAimManager().getRightHoodAngleDeg()));
    // CommandScheduler.getInstance()
    //     .schedule(
    //         robotContainer
    //             .getFlywheelLeft()
    //             .setRPMCommand(robotContainer.getTurretAimManager().getLeftFeederRPM()));
    // CommandScheduler.getInstance()
    //     .schedule(
    //         robotContainer
    //             .getFlywheelRight()
    //             .setRPMCommand(robotContainer.getTurretAimManager().getRightFeederRPM()));
  }

  // #endregion

  // #region state methods
  private boolean shouldTrack(BotState state) {
    return !inPit
        && state != BotState.DEFENCEIN
        && state != BotState.TRENCH
        && state != BotState.PIT;
  }

  private void onStateEntered(BotState state) {
    switch (state) {
      case SNOWBLOW:
        snowblow();
        break;
      case COLLECT:
        collect();
        break;
      case DEFENCEIN:
        defenseIn();
        break;
      case DEFENCEOUT:
        defenseOut();
        break;
      case DEPLOY:
        deploy();
        break;
      case RETRACT:
        retract();
        break;
      case TRENCH:
        trench();
        break;
      case PIT:
        // In pit mode, manual controls drive behavior.
        break;
    }
  }

  private void snowblow() {

    // Deploy intake to snowblow, and run motors to snowblow.
    CommandScheduler.getInstance().schedule(robotContainer.getIntakePivot().deployCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getAgitatorLeft().snowblowCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getAgitatorRight().snowblowCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getIntakeRollers().intakeCommand());
    CommandScheduler.getInstance()
        .schedule(robotContainer.getVerticalFeedLeft().verticalFeedIntakeCommand());
    CommandScheduler.getInstance()
        .schedule(robotContainer.getVerticalFeedRight().verticalFeedIntakeCommand());
  }

  private void collect() {

    // Deploy intake to collect, and run motors to intake and agitator motors.
    CommandScheduler.getInstance().schedule(robotContainer.getIntakePivot().deployCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getAgitatorLeft().collectCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getAgitatorRight().collectCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getIntakeRollers().intakeCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getVerticalFeedLeft().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getVerticalFeedRight().offCommand());
  }

  private void defenseIn() {

    // Retract intake to prevent damage, and stop all motors to save battery.
    CommandScheduler.getInstance().schedule(robotContainer.getIntakePivot().retractCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getAgitatorLeft().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getAgitatorRight().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getIntakeRollers().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getVerticalFeedLeft().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getVerticalFeedRight().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getTurretLeft().setDegreesCommand(0.0));
    CommandScheduler.getInstance().schedule(robotContainer.getTurretRight().setDegreesCommand(0.0));
    CommandScheduler.getInstance()
        .schedule(
            robotContainer
                .getHoodLeft()
                .setDegreesCommand(Constants.ScorerConstants.kHoodMinDegrees));
    CommandScheduler.getInstance()
        .schedule(
            robotContainer
                .getHoodRight()
                .setDegreesCommand(Constants.ScorerConstants.kHoodMinDegrees));
  }

  private void defenseOut() {
    CommandScheduler.getInstance().schedule(robotContainer.getIntakePivot().deployCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getFlywheelLeft().shootCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getFlywheelRight().shootCommand());
    // Enter DEFENCEOUT with intake deployed and all intake/feed rollers off.
    CommandScheduler.getInstance().schedule(robotContainer.getIntakeRollers().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getAgitatorLeft().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getAgitatorRight().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getVerticalFeedLeft().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getVerticalFeedRight().offCommand());
  }

  private void deploy() {
    CommandScheduler.getInstance().schedule(robotContainer.getIntakePivot().deployCommand());
  }

  private void retract() {
    CommandScheduler.getInstance().schedule(robotContainer.getIntakePivot().retractCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getIntakeRollers().offCommand());
  }

  private void trench() {
    CommandScheduler.getInstance()
        .schedule(
            robotContainer
                .getHoodLeft()
                .setDegreesCommand(Constants.ScorerConstants.kHoodStowedDegrees));
    CommandScheduler.getInstance()
        .schedule(
            robotContainer
                .getHoodRight()
                .setDegreesCommand(Constants.ScorerConstants.kHoodStowedDegrees));
  }

  // #endregion
}
