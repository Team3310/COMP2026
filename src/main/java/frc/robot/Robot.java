// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.util.FieldConstants;
import frc.lib.util.FieldConstants.Zone;
import frc.robot.Auton.AutonCommandBase;
import frc.robot.Auton.Paths;
import frc.robot.subsystems.Lights;
import frc.robot.util.choosers.AutonomousChooser;
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
  private AutonCommandBase cachedAutoCommand;
  private AutonomousChooser.AutonomousMode cachedAutoMode;
  private RobotContainer robotContainer;

  public static boolean inPit = false;

  public static BotState currentState = BotState.DEFENCEIN;
  public static OverrideState overrideState = OverrideState.OFF;
  public static boolean stateRefreshRequested = false;
  public static boolean crossOverrideActive = false;
  /** When true, turrets are locked to 180° + per-side offsets instead of aim-manager values. */
  public static boolean turretOverrideActive = false;

  public static boolean deploying = false;
  public static boolean retracting = false;
  public static boolean activeHub = true;
  public static boolean shootButtonHeld = false;
  public static boolean hubOverride = false; // true = manually forced OFF by SmartDashboard button
  public static Alliance currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
  public static FieldConstants.Zone currentZone = FieldConstants.Zone.BLUE;
  public static Lights.LightMode currentLightMode = Lights.LightMode.OFF;
  private BotState lastAppliedState = null;
  private Command snowblowGateCommand = null;

  // Dashboard read rate-limiting — tuning values don't need 50 Hz updates.
  // Read every Nth cycle to reduce NT traffic without affecting robot functionality.
  private int dashboardReadCounter = 0;
  private static final int DASHBOARD_READ_INTERVAL = 10; // Every 10th cycle (~5 Hz)
  // Dashboard write rate-limiting — telemetry doesn't need 50 Hz updates either.
  private int dashboardWriteCounter = 0;

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
    Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());
    Logger.recordOutput("Robot/DetectedBot", Constants.currentBot.name());
    Logger.recordOutput("Robot/LocalMacAddresses", Constants.getLocalMacAddressesString());

    // Refresh alliance color every cycle — DriverStation data may not be
    // available at class-load time, so the initial value can be wrong.
    currentAlliance = DriverStation.getAlliance().orElse(currentAlliance);

    double robotX = robotContainer.getDrive().getPose().getX();
    double robotY = robotContainer.getDrive().getPose().getY();
    double robotOrient = robotContainer.getDrive().getPose().getRotation().getDegrees();

    updateZone();

    // activeHub is still updated here because the broader robot state machine
    // still references it, but teleop drive slowdown no longer depends on
    // FMS/hub state.
    // updateHub();

    // This basically says if we are not deploying or retracting, then we can change
    // states.
    // If we are deploying or retracting, we want to stay in deploy or retract until
    // we are done.
    if (inPit && !robotContainer.pitOperatorMirrorsNormalMode) {
      currentState = BotState.PIT;
    } else if (!deploying && !retracting) {
      switch (overrideState) {
        case OFF: // nothing runs
          currentState = BotState.DEFENCEIN;
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
          Alliance effectiveAlliance = getEffectiveAlliance();
          if (currentZone == Zone.BLUE && effectiveAlliance == Alliance.Blue && !(activeHub)) {
            currentState = BotState.COLLECT;
          } else if (currentZone == Zone.RED && effectiveAlliance == Alliance.Red && !(activeHub)) {
            currentState = BotState.COLLECT;
          } else {
            currentState = BotState.SNOWBLOW;
          }
      }
    }

    if (stateRefreshRequested || currentState != lastAppliedState) {
      onStateEntered(currentState);
      lastAppliedState = currentState;
      stateRefreshRequested = false;
    }

    track();

    // Read speed/vision tuning overrides at ~5 Hz instead of 50 Hz to reduce NT traffic
    if (dashboardReadCounter++ % DASHBOARD_READ_INTERVAL == 0) {
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
      Constants.kLeftHoodConfig.kMaxPositionUnits = Constants.ScorerConstants.kHoodMaxDegrees;
      Constants.kRightHoodConfig.kMaxPositionUnits = Constants.ScorerConstants.kHoodMaxDegrees;
      Constants.ScorerConstants.kTurretMaxPositionUnits =
          SmartDashboard.getNumber(
              "SpeedTune/TurretMaxDegrees", Constants.ScorerConstants.kTurretMaxPositionUnits);
      Constants.ScorerConstants.kPhaseDelaySeconds =
          SmartDashboard.getNumber(
              "SpeedTune/PhaseDelaySeconds", Constants.ScorerConstants.kPhaseDelaySeconds);
      Constants.ScorerConstants.kReleaseDelaySeconds =
          SmartDashboard.getNumber(
              "SpeedTune/ReleaseDelaySeconds", Constants.ScorerConstants.kReleaseDelaySeconds);
      Constants.ScorerConstants.kTurretDeadbandDeg =
          SmartDashboard.getNumber(
              "SpeedTune/TurretDeadbandDeg", Constants.ScorerConstants.kTurretDeadbandDeg);
      // The old activeHub-based drive slowdown tuning is intentionally
      // commented out. Teleop drive mode selection now keys off alliance home
      // zone + shooting instead of FMS hub state.
      // Constants.DriveCommandConstants.kHubDriveScalar =
      //     SmartDashboard.getNumber(
      //         "SpeedTune/HubDriveScalar", Constants.DriveCommandConstants.kHubDriveScalar);
      // Constants.DriveCommandConstants.kHubTurnScalar =
      //     SmartDashboard.getNumber(
      //         "SpeedTune/HubTurnScalar", Constants.DriveCommandConstants.kHubTurnScalar);

      // Vision filter-strength overrides
      Constants.VisionConstants.kMT2StdDevMultiplier =
          SmartDashboard.getNumber(
              "VisionTune/MT2StdDevMultiplier", Constants.VisionConstants.kMT2StdDevMultiplier);
      Constants.VisionConstants.kMT1StdDevMultiplier =
          SmartDashboard.getNumber(
              "VisionTune/MT1StdDevMultiplier", Constants.VisionConstants.kMT1StdDevMultiplier);
      Constants.VisionConstants.kMaxPoseJumpMeters =
          SmartDashboard.getNumber(
              "VisionTune/MaxPoseJumpM", Constants.VisionConstants.kMaxPoseJumpMeters);
      Constants.VisionConstants.kMT2MaxAcceptedStdDev =
          SmartDashboard.getNumber(
              "VisionTune/MT2MaxAcceptedStdDev", Constants.VisionConstants.kMT2MaxAcceptedStdDev);
      Constants.ScorerConstants.kTurretOffsetDegrees =
          SmartDashboard.getNumber("turret offset", Constants.ScorerConstants.kTurretOffsetDegrees);
      Constants.ScorerConstants.kLeftTurretOffset =
          SmartDashboard.getNumber(
              "left turret offset", Constants.ScorerConstants.kLeftTurretOffset);
      Constants.ScorerConstants.kRightTurretOffset =
          SmartDashboard.getNumber(
              "right turret offset", Constants.ScorerConstants.kRightTurretOffset);
    }

    if (dashboardWriteCounter++ % DASHBOARD_READ_INTERVAL == 0) {
      SmartDashboard.putBoolean("inPit", inPit);
      SmartDashboard.putBoolean("PitMode/Active", inPit);
      SmartDashboard.putString("PitMode/Status", inPit ? "IN PIT" : "NORMAL");
      SmartDashboard.putString("currentState", "" + currentState);
      SmartDashboard.putString("overrideState", "" + overrideState);
      SmartDashboard.putNumber("robotX", robotX);
      SmartDashboard.putNumber("robotY", robotY);
      SmartDashboard.putNumber("robotOrient", robotOrient);
      SmartDashboard.putString("currentZone", "" + currentZone);
      SmartDashboard.putString("currentAlliance", "" + currentAlliance);
      SmartDashboard.putBoolean("activeHub", activeHub);
      SmartDashboard.putBoolean("hubOverride", hubOverride);
      SmartDashboard.putBoolean("crossOverrideActive", crossOverrideActive);
      SmartDashboard.putNumber("matchTime", DriverStation.getMatchTime());
      SmartDashboard.putBoolean("normalFlywheels", robotContainer.normalFlywheelsEnabled);
      SmartDashboard.putBoolean("pitFlywheels", robotContainer.pitFlywheelsEnabled);

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
      double totalMotorCurrent =
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
              + iTurretR;
      double totalDriveCurrent = robotContainer.getDrive().getTotalDriveTrainCurrentAmps();

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
      SmartDashboard.putNumber("Current/TotalMotors_A", totalMotorCurrent);
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
      SmartDashboard.putNumber("Current/TotalDrivetrain_A", totalDriveCurrent);
      SmartDashboard.putNumber("Current/TotalRobot_A", totalMotorCurrent + totalDriveCurrent);
    }

    // Return to non-RT thread priority (do not modify the first argument)
    // Threads.setCurrentThreadPriority(false, 10);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    var selectedAuto = robotContainer.getAutonomousChooser().getSelectedMode();
    robotContainer.getVision().setVisionEnabled(!selectedAuto.disablesVisionSeeding());
    if (selectedAuto.requiresPathLoading()) {
      Paths.loadPaths();
    }
    // Pre-build the auto command so autonomousInit() is instant.
    // Rebuild whenever the chooser selection changes.
    if (Paths.loaded && selectedAuto != cachedAutoMode) {
      cachedAutoCommand = selectedAuto.getCommand();
      cachedAutoMode = selectedAuto;
    }
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    inPit = false;
    currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);

    var selectedAuto = robotContainer.getAutonomousChooser().getSelectedMode();
    robotContainer.getVision().setVisionEnabled(!selectedAuto.disablesVisionSeeding());

    // Use the pre-built command from disabledPeriodic, or build now as fallback.
    AutonCommandBase autoCommand =
        (cachedAutoMode == selectedAuto && cachedAutoCommand != null)
            ? cachedAutoCommand
            : selectedAuto.getCommand();
    Rotation2d startingRotation = autoCommand.getStartingRotation();
    if (startingRotation != null) {
      Pose2d seededPose =
          new Pose2d(robotContainer.getDrive().getPose().getTranslation(), startingRotation);
      robotContainer.getDrive().setPose(seededPose);

      // Push the same heading to every Limelight so MegaTag 2's IMU is
      // correctly seeded at the moment auto begins.
      double yawDeg = startingRotation.getDegrees();
      for (String name : Constants.VisionConstants.kCameraNames) {
        LimelightHelpers.SetRobotOrientation(name, yawDeg, 0.0, 0.0, 0.0, 0.0, 0.0);
      }
    }

    robotContainer.getDrive().lockGyroHeadingToEstimatedPose();

    autonomousCommand = autoCommand;
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
    stateRefreshRequested = true;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    robotContainer.getDrive().lockGyroHeadingToEstimatedPose();
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    currentAlliance = DriverStation.getAlliance().orElse(Alliance.Blue);
    robotContainer.getVision().setVisionEnabled(true);
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    stateRefreshRequested = true;
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
    } else if (robotX < FieldConstants.Zone.MID.getX()) {
      currentZone = Zone.MID;
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
    if (matchTime < 0 || (gameData == null || gameData.isEmpty())) {
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

  private void track() {
    if (turretOverrideActive) {
      CommandScheduler.getInstance()
          .schedule(
              robotContainer
                  .getTurretLeft()
                  .setDegreesCommand(() -> 180.0 + Constants.ScorerConstants.kLeftTurretOffset));
      CommandScheduler.getInstance()
          .schedule(
              robotContainer
                  .getTurretRight()
                  .setDegreesCommand(() -> 180.0 + Constants.ScorerConstants.kRightTurretOffset));
    } else {
      CommandScheduler.getInstance()
          .schedule(
              robotContainer
                  .getTurretLeft()
                  .setDegreesCommand(robotContainer.getTurretAimManager().getLeftTurretAngleDeg()));
      CommandScheduler.getInstance()
          .schedule(
              robotContainer
                  .getTurretRight()
                  .setDegreesCommand(
                      robotContainer.getTurretAimManager().getRightTurretAngleDeg()));
    }
    if (shouldTrack(currentState) && !crossOverrideActive) {
      // Keep the hood at min degrees unless actively snowblowing or the
      // shoot button is held — prevents hood from tracking while driving.
      boolean holdMin = currentState != BotState.SNOWBLOW && !shootButtonHeld;
      double leftHoodDeg =
          holdMin
              ? Constants.ScorerConstants.kHoodMinDegrees
              : robotContainer.getTurretAimManager().getLeftHoodAngleDeg();
      double rightHoodDeg =
          holdMin
              ? Constants.ScorerConstants.kHoodMinDegrees
              : robotContainer.getTurretAimManager().getRightHoodAngleDeg();
      CommandScheduler.getInstance()
          .schedule(robotContainer.getHoodLeft().setDegreesCommand(leftHoodDeg));
      CommandScheduler.getInstance()
          .schedule(robotContainer.getHoodRight().setDegreesCommand(rightHoodDeg));
    }

    // Flywheel control in pit mode only — normal-mode flywheels are now
    // commanded by the operator right trigger (spin-up → feed sequence).
    if (inPit && !robotContainer.pitOperatorMirrorsNormalMode) {
      if (robotContainer.pitFlywheelsEnabled) {
        double pitRpm = Constants.ScorerConstants.kShootRPM;
        CommandScheduler.getInstance()
            .schedule(robotContainer.getFlywheelLeft().setRPMCommand(pitRpm));
        CommandScheduler.getInstance()
            .schedule(robotContainer.getFlywheelRight().setRPMCommand(pitRpm));
      } else {
        CommandScheduler.getInstance().schedule(robotContainer.getFlywheelLeft().offCommand());
        CommandScheduler.getInstance().schedule(robotContainer.getFlywheelRight().offCommand());
      }
    } else if (currentState == BotState.SNOWBLOW) {
      double leftTargetRpm = robotContainer.getTurretAimManager().getLeftFlywheelRPM();
      double rightTargetRpm = robotContainer.getTurretAimManager().getRightFlywheelRPM();
      CommandScheduler.getInstance()
          .schedule(robotContainer.getFlywheelLeft().setRPMCommand(leftTargetRpm));
      CommandScheduler.getInstance()
          .schedule(robotContainer.getFlywheelRight().setRPMCommand(rightTargetRpm));
    }
  }

  // #endregion

  // #region state methods
  private boolean shouldTrack(BotState state) {
    return (robotContainer.pitOperatorMirrorsNormalMode || !inPit)
        && state != BotState.DEFENCEIN
        && state != BotState.PIT;
  }

  public static boolean shouldLimitHomeZoneDrive() {
    return Robot.currentState == BotState.SNOWBLOW
        || (shootButtonHeld
            && ((getEffectiveAlliance() == Alliance.Blue && currentZone == Zone.BLUE)
                || (getEffectiveAlliance() == Alliance.Red && currentZone == Zone.RED)));
  }

  public static Alliance getEffectiveAlliance() {
    return inPit ? Alliance.Blue : currentAlliance;
  }

  private void onStateEntered(BotState state) {
    // Cancel the snowblow turret-gate loop so it stops re-scheduling
    // agitator/vertical-feed commands after we leave SNOWBLOW.
    if (snowblowGateCommand != null) {
      snowblowGateCommand.cancel();
      snowblowGateCommand = null;
    }
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
      case PIT:
        // In pit mode, manual controls drive behavior.
        break;
    }
  }

  private void snowblow() {

    // Deploy intake to snowblow, and run motors to snowblow.
    deploy();
    CommandScheduler.getInstance().schedule(robotContainer.getRoof().setMinHeightCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getIntakeRollers().intakeCommand());
    // Gate agitators and vertical feeds on turret position — off while the turret flips.
    java.util.function.DoubleSupplier feedRpm =
        robotContainer.getTurretAimManager()::getVerticalFeedRPM;
    snowblowGateCommand =
        Commands.run(
                () -> {
                  if (robotContainer.isTurretOnTarget()) {
                    CommandScheduler.getInstance()
                        .schedule(
                            robotContainer.getVerticalFeedLeft().customVelocityCommand(feedRpm));
                    CommandScheduler.getInstance()
                        .schedule(
                            robotContainer.getVerticalFeedRight().customVelocityCommand(feedRpm));
                    CommandScheduler.getInstance()
                        .schedule(robotContainer.getAgitatorLeft().snowblowCommand());
                    CommandScheduler.getInstance()
                        .schedule(robotContainer.getAgitatorRight().snowblowCommand());
                  } else {
                    CommandScheduler.getInstance()
                        .schedule(robotContainer.getVerticalFeedLeft().offCommand());
                    CommandScheduler.getInstance()
                        .schedule(robotContainer.getVerticalFeedRight().offCommand());
                    CommandScheduler.getInstance()
                        .schedule(robotContainer.getAgitatorLeft().offCommand());
                    CommandScheduler.getInstance()
                        .schedule(robotContainer.getAgitatorRight().offCommand());
                  }
                })
            .ignoringDisable(false);
    CommandScheduler.getInstance().schedule(snowblowGateCommand);
  }

  private void collect() {

    // Deploy intake to collect, and run motors to intake and agitator motors.
    deploy();
    CommandScheduler.getInstance().schedule(robotContainer.getRoof().setMaxHeightCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getFlywheelLeft().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getFlywheelRight().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getIntakeRollers().intakeCommand());
    CommandScheduler.getInstance()
        .schedule(robotContainer.getVerticalFeedLeft().verticalFeedCollectCommand());
    CommandScheduler.getInstance()
        .schedule(robotContainer.getVerticalFeedRight().verticalFeedCollectCommand());
  }

  private void defenseIn() {

    // Raise the roof before retracting the intake to avoid mechanism interference.
    CommandScheduler.getInstance()
        .schedule(
            robotContainer
                .getRoof()
                .motionMagicSetpointCommandBlocking(
                    () -> Constants.RoofConstants.kRoofMaxHeightInches, 0.25)
                .andThen(Commands.runOnce(this::retract)));
    CommandScheduler.getInstance().schedule(robotContainer.getFlywheelLeft().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getFlywheelRight().offCommand());
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
    deploy();
    // Enter DEFENCEOUT with intake deployed and all intake/feed rollers off.
    CommandScheduler.getInstance().schedule(robotContainer.getRoof().setMaxHeightCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getFlywheelLeft().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getFlywheelRight().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getIntakeRollers().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getAgitatorLeft().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getAgitatorRight().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getVerticalFeedLeft().offCommand());
    CommandScheduler.getInstance().schedule(robotContainer.getVerticalFeedRight().offCommand());
  }

  private void deploy() {
    if (robotContainer.getIntakePivot().getCurrentPosition() < 40.0) {
      CommandScheduler.getInstance().schedule(robotContainer.getIntakePivot().deployCommand());
      CommandScheduler.getInstance().schedule(robotContainer.getIntakeRollers().deployCommand());
    }
  }

  private void retract() {
    if (robotContainer.getIntakePivot().getCurrentPosition() > 40.0) {
      CommandScheduler.getInstance().schedule(robotContainer.getIntakePivot().retractCommand());
      CommandScheduler.getInstance().schedule(robotContainer.getIntakeRollers().retractCommand());
    }
  }

  // #endregion
}
