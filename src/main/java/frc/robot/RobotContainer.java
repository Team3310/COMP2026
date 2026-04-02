package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.subsystems.ServoMotorSubsystemConfig;
import frc.lib.subsystems.SimTalonFXIO;
import frc.lib.subsystems.TalonFXIO;
import frc.lib.util.Util;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Lights;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRollers;
// import frc.robot.subsystems.roof.Roof;
import frc.robot.subsystems.scorer.flywheel.Flywheel;
import frc.robot.subsystems.scorer.hood.Hood;
import frc.robot.subsystems.scorer.turret.Turret;
import frc.robot.subsystems.scorer.turret.TurretAimManager;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.choosers.AutonomousChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private static RobotContainer instance;

  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }
    return instance;
  }

  // Subsystem Builders
  private Drive buildDriveSystem() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      return new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));
    } else {
      return new Drive(
          new GyroIOSim(),
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight));
    }
  }

  private Agitator buildAgitatorSystem(ServoMotorSubsystemConfig config, int logOffset) {
    if (Constants.currentMode == Constants.Mode.REAL) {
      return new Agitator(config, new TalonFXIO(config), logOffset);
    } else {
      return new Agitator(config, new SimTalonFXIO(config), logOffset);
    }
  }

  private IntakeRollers buildIntakeRollersSystem() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      return new IntakeRollers(
          Constants.kIntakeRollerConfig, new TalonFXIO(Constants.kIntakeRollerConfig));
    } else {
      return new IntakeRollers(
          Constants.kIntakeRollerConfig, new SimTalonFXIO(Constants.kIntakeRollerConfig));
    }
  }

  private IntakePivot buildIntakePivotSystem() {
    if (Constants.currentMode == Constants.Mode.REAL) {
      return new IntakePivot(
          Constants.kIntakePivotConfig, new TalonFXIO(Constants.kIntakePivotConfig));
    } else {
      return new IntakePivot(
          Constants.kIntakePivotConfig, new SimTalonFXIO(Constants.kIntakePivotConfig));
    }
  }

  private Flywheel buildFlywheelSystem(ServoMotorSubsystemConfig config) {
    if (Constants.currentMode == Constants.Mode.REAL) {
      return new Flywheel(config, new TalonFXIO(config));
    } else {
      return new Flywheel(config, new SimTalonFXIO(config));
    }
  }

  private Hood buildHoodSystem(ServoMotorSubsystemConfig config) {
    if (Constants.currentMode == Constants.Mode.REAL) {
      return new Hood(config, new TalonFXIO(config));
    } else {
      return new Hood(config, new SimTalonFXIO(config));
    }
  }

  private Turret buildTurretSystem(ServoMotorSubsystemConfig config) {
    if (Constants.currentMode == Constants.Mode.REAL) {
      return new Turret(config, new TalonFXIO(config));
    } else {
      return new Turret(config, new SimTalonFXIO(config));
    }
  }

  // private Roof buildRoofSystem(ServoMotorSubsystemConfig config) {
  //   if (Constants.currentMode == Constants.Mode.REAL) {
  //     return new Roof(config, new TalonFXIO(config));
  //   } else {
  //     return new Roof(config, new SimTalonFXIO(config));
  //   }
  // }

  // Subsystem Intances
  private final Drive drive = buildDriveSystem();

  private final Agitator agitatorRight = buildAgitatorSystem(Constants.kRightFloorRollerConfig, 0);
  private final Agitator agitatorLeft = buildAgitatorSystem(Constants.kLeftFloorRollerConfig, 2);
  private final Agitator verticalFeedRight =
      buildAgitatorSystem(Constants.kRightVerticalFeedConfig, 5);
  private final Agitator verticalFeedLeft =
      buildAgitatorSystem(Constants.kLeftVerticalFeedConfig, 7);

  private final IntakeRollers intakeRollers = buildIntakeRollersSystem();
  private final IntakePivot intakePivot = buildIntakePivotSystem();

  private final Hood hoodLeft = buildHoodSystem(Constants.kLeftHoodConfig);
  private final Flywheel flywheelLeft = buildFlywheelSystem(Constants.kLeftFlywheelConfig);
  private final Turret turretLeft = buildTurretSystem(Constants.kLeftTurretConfig);

  private final Hood hoodRight = buildHoodSystem(Constants.kRightHoodConfig);
  private final Flywheel flywheelRight = buildFlywheelSystem(Constants.kRightFlywheelConfig);
  private final Turret turretRight = buildTurretSystem(Constants.kRightTurretConfig);

  // private final Roof roof = buildRoofSystem(Constants.kRoofConfig);

  public final AutonomousChooser autonomousChooser;

  // Turret aim calculator — computes desired turret/hood angles every cycle based
  // on robot pose, alliance color, and field zone. Logs everything via
  // AdvantageKit IO.
  private final TurretAimManager turretAimManager =
      new TurretAimManager(drive::getPose, drive::getChassisSpeeds);

  // Vision — 3× Limelight 4 cameras feeding MegaTag 2 poses into the drive
  // pose estimator.  Logs per-camera data under Vision/<cameraName>/ in
  // AdvantageScope.
  private final Vision vision = new Vision(drive);

  // #region getters
  public Drive getDrive() {
    return drive;
  }

  public Agitator getAgitatorRight() {
    return agitatorRight;
  }

  public Agitator getAgitatorLeft() {
    return agitatorLeft;
  }

  public Agitator getVerticalFeedRight() {
    return verticalFeedRight;
  }

  public Agitator getVerticalFeedLeft() {
    return verticalFeedLeft;
  }

  public IntakeRollers getIntakeRollers() {
    return intakeRollers;
  }

  public IntakePivot getIntakePivot() {
    return intakePivot;
  }

  public Hood getHoodLeft() {
    return hoodLeft;
  }

  public Flywheel getFlywheelLeft() {
    return flywheelLeft;
  }

  public Turret getTurretLeft() {
    return turretLeft;
  }

  public Hood getHoodRight() {
    return hoodRight;
  }

  public Flywheel getFlywheelRight() {
    return flywheelRight;
  }

  public double getDesiredLeftFlywheelRpm() {
    return desiredLeftFlywheelRpm;
  }

  public double getDesiredRightFlywheelRpm() {
    return desiredRightFlywheelRpm;
  }

  public void setDesiredFlywheelRpms(double leftRpm, double rightRpm) {
    desiredLeftFlywheelRpm = leftRpm;
    desiredRightFlywheelRpm = rightRpm;
  }

  public Turret getTurretRight() {
    return turretRight;
  }

  // public Roof getRoof() {
  //   return roof;
  // }

  public TurretAimManager getTurretAimManager() {
    return turretAimManager;
  }

  public Vision getVision() {
    return vision;
  }

  public AutonomousChooser getAutonomousChooser() {
    return autonomousChooser;
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand();
  }

  public Command buildAutoDefenseOutCommand() {
    return Commands.parallel(
            intakePivot.deployCommand().asProxy(),
            intakeRollers.offCommand().asProxy(),
            agitatorLeft.offCommand().asProxy(),
            verticalFeedLeft.offCommand().asProxy(),
            verticalFeedRight.offCommand().asProxy())
        .withName("Auto DefenceOut");
  }

  public Command buildAutoCollectCommand() {
    return Commands.parallel(
            intakePivot.deployCommand().asProxy(),
            intakeRollers.intakeCommand().asProxy(),
            agitatorLeft.collectCommand().asProxy(),
            // Right floor roller follows the left floor roller, so do not
            // directly command the follower in autonomous collect.
            verticalFeedLeft.verticalFeedCollectCommand().asProxy(),
            verticalFeedRight.verticalFeedCollectCommand().asProxy())
        .withName("Auto Collect");
  }

  public Command buildAutoSnowblowCommand() {
    java.util.function.DoubleSupplier leftTargetRpm = turretAimManager::getLeftFlywheelRPM;
    java.util.function.DoubleSupplier rightTargetRpm = turretAimManager::getRightFlywheelRPM;
    java.util.function.DoubleSupplier leftFeedRpm = turretAimManager::getLeftVerticalFeedRPM;
    java.util.function.DoubleSupplier rightFeedRpm = turretAimManager::getRightVerticalFeedRPM;

    return Commands.parallel(
            Commands.startEnd(
                () -> Robot.shootButtonHeld = true, () -> Robot.shootButtonHeld = false),
            flywheelLeft.setRPMCommand(leftTargetRpm).asProxy(),
            flywheelRight.setRPMCommand(rightTargetRpm).asProxy(),
            intakePivot.deployCommand().asProxy(),
            intakeRollers.intakeCommand().asProxy(),
            new WaitCommand(Constants.ScorerConstants.kWaitTime)
                .andThen(
                    Commands.parallel(
                        agitatorLeft.snowblowCommand().asProxy(),
                        // Right floor roller now follows the left floor roller.
                        // Keep the old direct command commented out so follower
                        // mode is not overridden.
                        // agitatorRight.snowblowCommand().asProxy(),
                        verticalFeedLeft.setRPMCommand(leftFeedRpm).asProxy(),
                        verticalFeedRight.setRPMCommand(rightFeedRpm).asProxy())))
        .withName("Auto Snowblow");
  }

  // #endregion

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(1);
  private final CommandXboxController operator = new CommandXboxController(0);
  private static final Pose2d kPitAssumedPose =
      new Pose2d(new Translation2d(3.581, 4.039), Rotation2d.kZero);
  // Flywheel enable policy:
  // - Pit mode: controlled by pitFlywheelsEnabled via operator/dashboard toggles.
  // - Normal mode: controlled by normalFlywheelsEnabled (default true for auto +
  // teleop).
  public boolean pitFlywheelsEnabled = false;
  public boolean normalFlywheelsEnabled = true;
  // Keep pit mode as a distinct path, but default operator pit behavior to mirror normal mode.
  public boolean pitOperatorMirrorsNormalMode = true;
  public boolean pitManualTurretEnabled = false;
  private double desiredLeftFlywheelRpm = 0.0;
  private double desiredRightFlywheelRpm = 0.0;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Right floor roller follows the left floor roller so the left remains
    // the only floor roller running VelocityTorqueCurrentFOC closed-loop control.
    agitatorRight.motorIO.follow(Constants.kLeftFloorRollerConfig.talonCANID, true);
    agitatorRight.setDefaultCommand(
        Commands.run(
                () ->
                    agitatorRight.motorIO.follow(Constants.kLeftFloorRollerConfig.talonCANID, true),
                agitatorRight)
            .ignoringDisable(true)
            .withName("RightFloorRoller Follow LeftFloorRoller"));

    // Register PathPlanner named commands (must be before any path loading)
    NamedCommands.registerCommand("switchToCollect", buildAutoCollectCommand());
    NamedCommands.registerCommand("switchToSnowblow", buildAutoSnowblowCommand());
    NamedCommands.registerCommand("snowblow", buildSnowblowCommand());
    NamedCommands.registerCommand("crossOverrde", buildCrossOverrideCommand());
    NamedCommands.registerCommand(
        "deployIntake",
        Commands.parallel(intakePivot.deployCommand(), intakeRollers.deployCommand()));

    // Initialize autonomous commands
    autonomousChooser = new AutonomousChooser();
    DriverReadout.addChoosers(autonomousChooser);

    // #region Dashboard Buttons
    SmartDashboard.putData(
        "enterPitMode",
        Commands.runOnce(
            () -> {
              Robot.inPit = true;
              drive.setPose(kPitAssumedPose);
            },
            drive));
    SmartDashboard.putData(
        "exitPitMode",
        Commands.runOnce(
            () -> {
              Robot.inPit = false;
            }));

    SmartDashboard.putData("intake", intakeRollers.intakeCommand());
    SmartDashboard.putData("outtake", intakeRollers.outtakeCommand());
    SmartDashboard.putData(
        "shoot",
        Commands.parallel(
            flywheelLeft.shootCommand(),
            flywheelRight.shootCommand(),
            verticalFeedLeft.verticalFeedIntakeCommand(),
            verticalFeedRight.verticalFeedIntakeCommand(),
            agitatorLeft.snowblowCommand(),
            // Right floor roller now follows the left floor roller.
            // Keep the old direct command commented out so follower mode is not overridden.
            // agitatorRight.snowblowCommand(),
            hoodLeft.setDegreesCommand(Constants.ScorerConstants.kHoodStowedDegrees),
            hoodRight.setDegreesCommand(Constants.ScorerConstants.kHoodStowedDegrees)));
    SmartDashboard.putData(
        "rotate Robot 0°",
        DriveCommands.joystickDriveAtAngle(
            drive, () -> 0.0, () -> 0.0, () -> Rotation2d.fromDegrees(0.0)));
    SmartDashboard.putData(
        "rotate Robot 90°",
        DriveCommands.joystickDriveAtAngle(
            drive, () -> 0.0, () -> 0.0, () -> Rotation2d.fromDegrees(90.0)));
    SmartDashboard.putData(
        "rotate Robot 180°",
        DriveCommands.joystickDriveAtAngle(
            drive, () -> 0.0, () -> 0.0, () -> Rotation2d.fromDegrees(180.0)));
    SmartDashboard.putData(
        "rotate Robot 270°",
        DriveCommands.joystickDriveAtAngle(
            drive, () -> 0.0, () -> 0.0, () -> Rotation2d.fromDegrees(270.0)));
    SmartDashboard.putData(
        "rotate left turret", turretLeft.setDegreesCommand(turretLeft.getCurrentPosition() + 90.0));
    SmartDashboard.putData(
        "rotate right turret",
        turretRight.setDegreesCommand(turretRight.getCurrentPosition() + 90.0));
    SmartDashboard.putData(
        "deploy intake",
        Commands.parallel(
            intakePivot.setDegreesCommand(Constants.IntakeConstants.kIntakePivotDeployDegrees),
            intakeRollers.deployCommand()));
    SmartDashboard.putData(
        "retract intake",
        intakePivot.setDegreesCommand(Constants.IntakeConstants.kIntakePivotStowedDegrees));
    SmartDashboard.putData(
        "floors on",
        Commands.parallel(
            agitatorLeft.snowblowCommand()
            // Right floor roller now follows the left floor roller.
            // Keep the old direct command commented out so follower mode is not overridden.
            // , agitatorRight.snowblowCommand()
            ));
    SmartDashboard.putData(
        "spit balls",
        Commands.parallel(
            agitatorLeft.reverseCommand(),
            // Right floor roller now follows the left floor roller.
            // Keep the old direct command commented out so follower mode is not overridden.
            // agitatorRight.reverseCommand(),
            intakeRollers.outtakeCommand()));
    SmartDashboard.putData(
        "vertical feed on",
        Commands.parallel(
            verticalFeedLeft.verticalFeedIntakeCommand(),
            verticalFeedRight.verticalFeedIntakeCommand()));
    SmartDashboard.putData(
        "vertical feed reverse",
        Commands.parallel(
            verticalFeedLeft.verticalFeedOuttakeCommand(),
            verticalFeedRight.verticalFeedOuttakeCommand()));
    SmartDashboard.putData(
        "flywheel on",
        Commands.runOnce(
            () -> {
              if (Robot.inPit) {
                pitFlywheelsEnabled = true;
              } else {
                normalFlywheelsEnabled = true;
              }
            }));
    SmartDashboard.putData(
        "flywheel off",
        Commands.runOnce(
            () -> {
              if (Robot.inPit) {
                pitFlywheelsEnabled = false;
              } else {
                normalFlywheelsEnabled = false;
              }
            }));
    SmartDashboard.putData(
        "Move Hood to max", Commands.parallel(hoodLeft.setMaxCommand(), hoodRight.setMaxCommand()));
    SmartDashboard.putData(
        "Reset Hood",
        Commands.parallel(
            hoodLeft.setDegreesCommand(Constants.ScorerConstants.kHoodStowedDegrees),
            hoodRight.setDegreesCommand(Constants.ScorerConstants.kHoodStowedDegrees)));
    SmartDashboard.putData(
        "rotate turret to max",
        Commands.parallel(turretLeft.maxCommand(), turretRight.maxCommand()));

    SmartDashboard.putData(
        "Change Hub Active", new InstantCommand(() -> Robot.hubOverride = !Robot.hubOverride));

    // SmartDashboard.putData("Roof Deploy", roof.setMaxHeightCommand());
    // SmartDashboard.putData("Roof Stow", roof.setMinHeightCommand());

    // LED on/off buttons
    // SmartDashboard.putData(
    //     "Lights ON",
    //     new InstantCommand(() -> Lights.getInstance().setMode(Lights.LightMode.BOT_STATE))
    //         .ignoringDisable(true));
    // SmartDashboard.putData(
    //     "Lights OFF",
    //     new InstantCommand(() -> Lights.getInstance().setMode(Lights.LightMode.OFF))
    //         .ignoringDisable(true));

    // Speed tuning — publish defaults so Elastic/SmartDashboard shows editable
    // number widgets.
    // Robot.robotPeriodic() reads these back into the Constants each loop.
    SmartDashboard.putNumber(
        "SpeedTune/FloorForwardRPM", Constants.AgitatorConstants.kFloorRollerSnowblowRPM);
    SmartDashboard.putNumber(
        "SpeedTune/FloorReverseRPM", Constants.AgitatorConstants.kFloorRollerReverseRPM);
    SmartDashboard.putNumber(
        "SpeedTune/IntakeForwardRPM", Constants.IntakeConstants.kIntakeVelocityRPM);
    SmartDashboard.putNumber(
        "SpeedTune/IntakeReverseRPM", Constants.IntakeConstants.kOuttakeVelocityRPM);
    SmartDashboard.putNumber(
        "SpeedTune/VertFeedForwardRPM", Constants.AgitatorConstants.kVerticalFeedIntakeRPM);
    SmartDashboard.putNumber(
        "SpeedTune/VertFeedReverseRPM", Constants.AgitatorConstants.kVerticalFeedOuttakeRPM);
    SmartDashboard.putNumber("SpeedTune/FlywheelForwardRPM", Constants.ScorerConstants.kShootRPM);
    SmartDashboard.putNumber(
        "SpeedTune/FlywheelReverseRPM", Constants.ScorerConstants.kReverseShootRPM);
    SmartDashboard.putNumber(
        "SpeedTune/TurretMaxDegrees", Constants.ScorerConstants.kTurretMaxPositionUnits);
    SmartDashboard.putNumber("SpeedTune/HoodMaxDegrees", Constants.ScorerConstants.kHoodMaxDegrees);
    SmartDashboard.putNumber(
        "SpeedTune/PhaseDelaySeconds", Constants.ScorerConstants.kPhaseDelaySeconds);
    SmartDashboard.putNumber(
        "SpeedTune/ReleaseDelaySeconds", Constants.ScorerConstants.kReleaseDelaySeconds);

    // Vision filter-strength tuning — multipliers for the Limelight std devs
    // fed into the WPILib pose estimator.  >1.0 = smoother, <1.0 = snappier.
    SmartDashboard.putNumber(
        "VisionTune/MT2StdDevMultiplier", Constants.VisionConstants.kMT2StdDevMultiplier);
    SmartDashboard.putNumber(
        "VisionTune/MT1StdDevMultiplier", Constants.VisionConstants.kMT1StdDevMultiplier);
    SmartDashboard.putNumber(
        "VisionTune/MaxPoseJumpM", Constants.VisionConstants.kMaxPoseJumpMeters);
    SmartDashboard.putNumber(
        "VisionTune/MT2MaxAcceptedStdDev", Constants.VisionConstants.kMT2MaxAcceptedStdDev);
    SmartDashboard.putNumber("turret offset", Constants.ScorerConstants.kTurretOffsetDegrees);

    // Removed individual turret offset controls; only the global turret offset remains adjustable.

    // Light color buttons — work even while disabled
    // SmartDashboard.putData(
    //     "Lights Red",
    //     new InstantCommand(() -> Lights.getInstance().setMode(Lights.LightMode.COLOR_RED))
    //         .ignoringDisable(true));
    // SmartDashboard.putData(
    //     "Lights Blue",
    //     new InstantCommand(() -> Lights.getInstance().setMode(Lights.LightMode.COLOR_BLUE))
    //         .ignoringDisable(true));
    // SmartDashboard.putData(
    //     "Lights Green",
    //     new InstantCommand(() -> Lights.getInstance().setMode(Lights.LightMode.COLOR_GREEN))
    //         .ignoringDisable(true));
    // SmartDashboard.putData(
    //     "Lights Orange",
    //     new InstantCommand(() -> Lights.getInstance().setMode(Lights.LightMode.COLOR_ORANGE))
    //         .ignoringDisable(true));
    // #endregion

    // Initialize LED display mode to show bot state colors
    // Lights.getInstance().setMode(Lights.LightMode.BOT_STATE);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private Command buildNormalSnapCommand(double targetAngleDeg) {
    return DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> getDriverPerspectiveSnapAngle(targetAngleDeg))
        .until(
            () ->
                Math.abs(driver.getRightX())
                    > Constants.DriveCommandConstants.kRotationCommandDeadband);
  }

  private Rotation2d getDriverPerspectiveSnapAngle(double blueFrameAngleDeg) {
    Rotation2d targetAngle = Rotation2d.fromDegrees(blueFrameAngleDeg);
    return Robot.currentAlliance == Alliance.Red ? Util.flipRedBlue(targetAngle) : targetAngle;
  }

  private Command buildCrossOverrideCommand() {
    return Commands.parallel(
            hoodLeft.setDegreesCommand(Constants.ScorerConstants.kHoodMinDegrees),
            hoodRight.setDegreesCommand(Constants.ScorerConstants.kHoodMinDegrees),
            intakePivot.setDegreesCommand(Constants.IntakeConstants.kHeadButtDegrees))
        .alongWith(
            Commands.startEnd(
                () -> Robot.crossOverrideActive = true, () -> Robot.crossOverrideActive = false));
  }

  private Command buildOverrideStateCommand(Robot.OverrideState overrideState) {
    return Commands.runOnce(
        () -> {
          Robot.overrideState = overrideState;
          Robot.stateRefreshRequested = true;
        });
  }

  /**
   * Waits until both floor rollers are at snowblow speed (live from the SmartDashboard table), then
   * enables both vertical feed rollers at the aim-manager's desired RPM. Registered as the
   * PathPlanner named command {@code "snowblow"}.
   */
  private Command buildSnowblowCommand() {
    return new edu.wpi.first.wpilibj2.command.WaitUntilCommand(
            () -> {
              double target = Constants.AgitatorConstants.kFloorRollerSnowblowRPM;
              double tol = Constants.ScorerConstants.kFlywheelRPMTolerance;
              return Math.abs(agitatorLeft.getCurrentVelocity() - target) < tol
                  && Math.abs(agitatorRight.getCurrentVelocity() - target) < tol;
            })
        .withTimeout(3.0)
        .andThen(
            Commands.runOnce(
                () -> {
                  java.util.function.DoubleSupplier leftFeedRpm =
                      turretAimManager::getLeftVerticalFeedRPM;
                  java.util.function.DoubleSupplier rightFeedRpm =
                      turretAimManager::getRightVerticalFeedRPM;
                  edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance()
                      .schedule(verticalFeedLeft.setRPMCommand(leftFeedRpm));
                  edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance()
                      .schedule(verticalFeedRight.setRPMCommand(rightFeedRpm));
                }))
        .withName("SnowblowEnableVertical");
  }

  private boolean isOperatorJamClearOverrideAllowed() {
    return isUsingLegacyPitOperatorMode()
        || Robot.currentState == Robot.BotState.DEFENCEOUT
        || Robot.currentState == Robot.BotState.COLLECT;
  }

  private Command buildOperatorJamClearOverrideCommand() {
    return Commands.parallel(
            intakeRollers.outtakeCommand(),
            agitatorLeft.reverseCommand(),
            // Right floor roller now follows the left floor roller.
            // Keep the old direct command commented out so follower mode is not overridden.
            // agitatorRight.reverseCommand(),
            verticalFeedLeft.verticalFeedOuttakeCommand(),
            verticalFeedRight.verticalFeedOuttakeCommand())
        .finallyDo(() -> Robot.stateRefreshRequested = true);
  }

  private boolean isUsingLegacyPitOperatorMode() {
    return Robot.inPit && !pitOperatorMirrorsNormalMode;
  }

  private Command buildPitCardinalDriveCommand(
      double xMetersPerSecondScalar, double yMetersPerSecondScalar) {
    return DriveCommands.joystickDrive(
        drive, () -> xMetersPerSecondScalar, () -> yMetersPerSecondScalar, () -> 0.0);
  }

  /**
   * Returns true when both physical turrets are within tolerance of their commanded positions. Use
   * this to gate feeders so balls aren't launched while the turret is mid-flip.
   */
  public boolean isTurretOnTarget() {
    double tol = Constants.ScorerConstants.kTurretLockOnToleranceDeg;
    double leftErr =
        Math.abs(turretLeft.getCurrentPosition() - turretAimManager.getLeftTurretAngleDeg());
    double rightErr =
        Math.abs(turretRight.getCurrentPosition() - turretAimManager.getRightTurretAngleDeg());
    return leftErr < tol && rightErr < tol;
  }

  private Command buildShootWhileHeldCommand() {
    java.util.function.DoubleSupplier leftTargetRpm = turretAimManager::getLeftFlywheelRPM;
    java.util.function.DoubleSupplier rightTargetRpm = turretAimManager::getRightFlywheelRPM;
    java.util.function.DoubleSupplier leftVerticalFeedTargetRpm =
        turretAimManager::getLeftVerticalFeedRPM;
    java.util.function.DoubleSupplier rightVerticalFeedTargetRpm =
        turretAimManager::getRightVerticalFeedRPM;

    return Commands.parallel(
        Commands.startEnd(() -> Robot.shootButtonHeld = true, () -> Robot.shootButtonHeld = false),
        Commands.parallel(
                flywheelLeft.setRPMCommand(leftTargetRpm),
                flywheelRight.setRPMCommand(rightTargetRpm))
            .alongWith(
                new WaitCommand(Constants.ScorerConstants.kWaitTime)
                    .andThen(
                        // Use Commands.parallel so subsystem requirements are properly held
                        // and commands stay running — never schedule() inside run().
                        Commands.parallel(
                            verticalFeedLeft.setRPMCommand(leftVerticalFeedTargetRpm),
                            verticalFeedRight.setRPMCommand(rightVerticalFeedTargetRpm),
                            agitatorLeft.snowblowCommand()
                            // Right floor roller now follows the left floor roller.
                            // Keep the old direct command commented out so follower mode is not
                            // overridden.
                            // , agitatorRight.snowblowCommand()
                            )))
            .finallyDo(
                () -> {
                  CommandScheduler.getInstance().schedule(flywheelLeft.offCommand());
                  CommandScheduler.getInstance().schedule(flywheelRight.offCommand());
                  CommandScheduler.getInstance().schedule(verticalFeedLeft.offCommand());
                  CommandScheduler.getInstance().schedule(verticalFeedRight.offCommand());
                  CommandScheduler.getInstance().schedule(agitatorLeft.offCommand());
                  // Right floor roller now follows the left floor roller.
                  // Keep the old direct off command commented out so follower mode is not
                  // overridden.
                  // CommandScheduler.getInstance().schedule(agitatorRight.offCommand());
                  Robot.stateRefreshRequested = true;
                }));
  }

  private void configureButtonBindings() {
    // Flywheel defaults enforce desired mode behavior:
    // - Pit: off unless explicitly enabled.
    // - Normal: on unless explicitly disabled.
    // Flywheels off when:
    //  - Pit mode and pitFlywheelsEnabled is false
    //  - Normal mode and normalFlywheelsEnabled is false
    //  - DEFENCEIN (hood at min / "cross" - do not shoot)]

    // Hood defaults: continuously hold the last-commanded position via motion
    // magic.  Without this the base-class neutral command takes over as soon as a
    // setDegreesCommand finishes, and the hood drifts back to zero / goes limp.
    hoodLeft.setTeleopDefaultCommand();
    hoodRight.setTeleopDefaultCommand();
    // roof.setTeleopDefaultCommand();
    flywheelLeft.setDefaultCommand(
        flywheelLeft.offCommand().withName("Flywheel Left Neutral (default)"));
    flywheelRight.setDefaultCommand(
        flywheelRight.offCommand().withName("Flywheel Right Neutral (default)"));

    // Default command, normal field-relative drive (same in both modes)
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // #region Driver Controls
    // --- DRIVER BINDINGS
    // -----------------------------------------------------------------------

    // Face buttons (pit): hold to drive full-speed cardinals.
    driver.y().and(() -> Robot.inPit).whileTrue(buildPitCardinalDriveCommand(1.0, 0.0));
    driver.x().and(() -> Robot.inPit).whileTrue(buildPitCardinalDriveCommand(0.0, 1.0));
    driver.a().and(() -> Robot.inPit).whileTrue(buildPitCardinalDriveCommand(-1.0, 0.0));
    driver.b().and(() -> Robot.inPit).whileTrue(buildPitCardinalDriveCommand(0.0, -1.0));

    // Face buttons (normal): on release, snap to angle and keep holding until
    // driver commands
    // manual rotation by pushing right-stick X outside the normal rotation
    // deadband.
    driver
        .y()
        .and(() -> !Robot.inPit)
        .onFalse(buildNormalSnapCommand(Constants.DriveCommandConstants.kDriverSnapAngleYDeg));
    driver
        .x()
        .and(() -> !Robot.inPit)
        .onFalse(buildNormalSnapCommand(Constants.DriveCommandConstants.kDriverSnapAngleXDeg));
    driver
        .a()
        .and(() -> !Robot.inPit)
        .onFalse(buildNormalSnapCommand(Constants.DriveCommandConstants.kDriverSnapAngleADeg));
    driver
        .b()
        .and(() -> !Robot.inPit)
        .onFalse(buildNormalSnapCommand(Constants.DriveCommandConstants.kDriverSnapAngleBDeg));

    // Start: normal = zero gyro (both modes)
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Back: pit = set pose to Blue right-of-hub scoring position, normal = set pose to behind Red
    // Hub
    driver
        .back()
        .and(() -> Robot.inPit)
        .onTrue(
            Commands.runOnce(() -> drive.setPose(kPitAssumedPose), drive).ignoringDisable(true));
    driver
        .back()
        .and(() -> !Robot.inPit)
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Pose2d blueFrontHubPose =
                          new Pose2d(new Translation2d(3.581, 4.039), Rotation2d.kZero);
                      Pose2d targetPose =
                          Robot.currentAlliance == Alliance.Red
                              ? new Pose2d(
                                  Util.flipRedBlue(blueFrontHubPose.getTranslation()),
                                  Util.flipRedBlue(blueFrontHubPose.getRotation()))
                              : blueFrontHubPose;
                      drive.setPose(targetPose);
                    },
                    drive)
                .ignoringDisable(true));

    // right bumper = unbound

    // left bumper = intake roller outtake only while held in both modes
    driver.leftBumper().whileTrue(intakeRollers.outtakeCommand());

    // right trigger = unbound

    // left trigger = intake roller only while held in both modes
    driver.leftTrigger().whileTrue(intakeRollers.intakeCommand());

    driver
        .povDown()
        .and(() -> !Robot.inPit)
        .onTrue(
            Commands.parallel(
                hoodLeft.setDegreesCommand(Constants.ScorerConstants.kHoodMinDegrees),
                hoodRight.setDegreesCommand(Constants.ScorerConstants.kHoodMinDegrees)));
    // #endregion

    // #region Operator Controls
    // ----------------------------------------------------------------------------------------------------

    if (turretLeft != null) {

      // Pit: left joystick angle maps directly to left turret angle.
      // The command is still clamped by the turret software limits (currently ±220°).
      // Only updates when stick is pushed past deadband magnitude.
      turretLeft.setDefaultCommand(
          turretLeft.dutyCycleCommand(
              () -> {
                if (!Robot.inPit) return 0.0;
                if (!pitManualTurretEnabled) return 0.0;
                double x = operator.getLeftX();
                double y = operator.getLeftY();
                if (Math.sqrt(x * x + y * y) < 0.5) return 0.0;
                double angleDeg = Math.toDegrees(Math.atan2(x, -y)); // 0° = stick up, +CW
                double clamped =
                    Math.max(
                        Constants.ScorerConstants.kTurretMinPositionUnits,
                        Math.min(Constants.ScorerConstants.kTurretMaxPositionUnits, angleDeg));
                edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance()
                    .schedule(turretLeft.setDegreesCommand(clamped));
                return 0.0;
              }));
    }

    if (turretRight != null) {
      // Pit: right joystick angle maps directly to right turret angle.
      // The command is still clamped by the turret software limits (currently ±220°).
      turretRight.setDefaultCommand(
          turretRight.dutyCycleCommand(
              () -> {
                if (!Robot.inPit) return 0.0;
                if (!pitManualTurretEnabled) return 0.0;
                double x = operator.getRightX();
                double y = operator.getRightY();
                if (Math.sqrt(x * x + y * y) < 0.5) return 0.0;
                double angleDeg = Math.toDegrees(Math.atan2(x, -y));
                double clamped =
                    Math.max(
                        Constants.ScorerConstants.kTurretMinPositionUnits,
                        Math.min(Constants.ScorerConstants.kTurretMaxPositionUnits, angleDeg));
                edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance()
                    .schedule(turretRight.setDegreesCommand(clamped));
                return 0.0;
              }));
    }

    if (agitatorRight != null && agitatorLeft != null) {
      // Y = pit legacy: shooter rollers on | normal and mirrored pit: snowblow state
      operator
          .y()
          .onTrue(
              Commands.either(
                  Commands.runOnce(() -> pitFlywheelsEnabled = true),
                  buildOverrideStateCommand(Robot.OverrideState.SNOWBLOW),
                  this::isUsingLegacyPitOperatorMode));
    }

    // X = pit legacy: shooter rollers off | normal and mirrored pit: defenceout state
    operator
        .x()
        .onTrue(
            Commands.either(
                Commands.runOnce(() -> pitFlywheelsEnabled = false),
                buildOverrideStateCommand(Robot.OverrideState.DEFENCEOUT),
                this::isUsingLegacyPitOperatorMode));

    // B = pit legacy: floors on | normal and mirrored pit: collect state
    operator
        .b()
        .toggleOnTrue(
            Commands.either(
                Commands.parallel(
                    agitatorLeft.snowblowCommand()
                    // Right floor roller now follows the left floor roller.
                    // Keep the old direct command commented out so follower mode is not overridden.
                    // , agitatorRight.snowblowCommand()
                    ),
                buildOverrideStateCommand(Robot.OverrideState.COLLECT),
                this::isUsingLegacyPitOperatorMode));

    // A = pit legacy: floors off | normal and mirrored pit: defencein state
    operator
        .a()
        .toggleOnTrue(
            Commands.either(
                Commands.parallel(
                    agitatorLeft.offCommand()
                    // Right floor roller now follows the left floor roller.
                    // Keep the old direct off command commented out so follower mode is not
                    // overridden.
                    // , agitatorRight.offCommand()
                    ),
                buildOverrideStateCommand(Robot.OverrideState.DEFENCEIN),
                this::isUsingLegacyPitOperatorMode));

    // Right Trigger:
    // Pit legacy -> toggle vertical feed rollers on/off.
    // Normal mode and mirrored pit -> spin flywheels to TurretAimManager RPM, wait for at-speed,
    // then engage vertical feeders + agitators. All off on release.
    operator
        .rightTrigger()
        .and(() -> !isUsingLegacyPitOperatorMode())
        .and(
            () ->
                Robot.currentState == Robot.BotState.DEFENCEOUT
                    || Robot.currentState == Robot.BotState.COLLECT)
        .whileTrue(buildShootWhileHeldCommand());

    // Left Trigger:
    // Pit legacy -> toggleOnTrue: vertical rollers off
    // Normal mode and mirrored pit -> whileTrue: intake on only while held in DEFENCEOUT
    if (verticalFeedRight != null && verticalFeedLeft != null) {
      operator
          .leftTrigger()
          .and(this::isUsingLegacyPitOperatorMode)
          .toggleOnTrue(
              Commands.parallel(verticalFeedLeft.offCommand(), verticalFeedRight.offCommand()));

      operator
          .leftTrigger()
          .and(
              () ->
                  !isUsingLegacyPitOperatorMode()
                      && Robot.currentState == Robot.BotState.DEFENCEOUT)
          .whileTrue(intakeRollers.intakeCommand());
    }

    // Left bumper = jam-clear override while held
    operator
        .leftBumper()
        .and(this::isOperatorJamClearOverrideAllowed)
        .whileTrue(buildOperatorJamClearOverrideCommand());

    if (flywheelLeft != null && hoodLeft != null) {
      Trigger crossOverrideTrigger = operator.povDown();

      // D-Pad Up/Down: same in normal and mirrored pit. Legacy pit keeps old preset behavior.
      operator
          .povUp()
          .onTrue(
              Commands.either(
                  Commands.parallel(
                      hoodLeft.setDegreesCommand(() -> Constants.ScorerConstants.kHoodMaxDegrees),
                      hoodRight.setDegreesCommand(() -> Constants.ScorerConstants.kHoodMaxDegrees)),
                  Commands.parallel(
                      hoodLeft.setDegreesCommand(() -> Constants.ScorerConstants.kHoodMaxDegrees),
                      hoodRight.setDegreesCommand(() -> Constants.ScorerConstants.kHoodMaxDegrees)),
                  this::isUsingLegacyPitOperatorMode));
      crossOverrideTrigger.whileTrue(buildCrossOverrideCommand());
      // Legacy pit: D-Pad Right = hoods to 25 deg, D-Pad Left = hoods to 20 deg
      operator
          .povRight()
          .onTrue(
              Commands.either(
                  Commands.parallel(
                      hoodLeft.setDegreesCommand(25.0), hoodRight.setDegreesCommand(25.0)),
                  Commands.none(),
                  this::isUsingLegacyPitOperatorMode));
      operator
          .povLeft()
          .onTrue(
              Commands.either(
                  Commands.parallel(
                      hoodLeft.setDegreesCommand(20.0), hoodRight.setDegreesCommand(20.0)),
                  Commands.none(),
                  this::isUsingLegacyPitOperatorMode));
    }
    // #endregion
  }
}
