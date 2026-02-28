package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.subsystems.ServoMotorSubsystemConfig;
import frc.lib.subsystems.SimTalonFXIO;
import frc.lib.subsystems.TalonFXIO;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.scorer.flywheel.Flywheel;
import frc.robot.subsystems.scorer.hood.Hood;
import frc.robot.subsystems.scorer.turret.Turret;
import frc.robot.subsystems.scorer.turret.TurretAimManager;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
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

  private Agitator buildAgitatorSystem(ServoMotorSubsystemConfig config) {
    if (Constants.currentMode == Constants.Mode.REAL) {
      return new Agitator(config, new TalonFXIO(config));
    } else {
      return new Agitator(config, new SimTalonFXIO(config));
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

  // Subsystem Intances
  private final Drive drive = buildDriveSystem();

  private final Agitator agitatorRight = buildAgitatorSystem(Constants.kRightFloorRollerConfig);
  private final Agitator agitatorLeft = buildAgitatorSystem(Constants.kLeftFloorRollerConfig);
  private final Agitator verticalFeedRight =
      buildAgitatorSystem(Constants.kRightVerticalFeedConfig);
  private final Agitator verticalFeedLeft = buildAgitatorSystem(Constants.kLeftVerticalFeedConfig);

  private final IntakeRollers intakeRollers = buildIntakeRollersSystem();
  private final IntakePivot intakePivot = buildIntakePivotSystem();

  private final Hood hoodLeft = buildHoodSystem(Constants.kLeftHoodConfig);
  private final Flywheel flywheelLeft = buildFlywheelSystem(Constants.kLeftFlywheelConfig);
  private final Turret turretLeft = buildTurretSystem(Constants.kLeftTurretConfig);

  private final Hood hoodRight = buildHoodSystem(Constants.kRightHoodConfig);
  private final Flywheel flywheelRight = buildFlywheelSystem(Constants.kRightFlywheelConfig);
  private final Turret turretRight = buildTurretSystem(Constants.kRightTurretConfig);

  // Turret aim calculator — computes desired turret/hood angles every cycle based
  // on robot pose, alliance color, and field zone. Logs everything via
  // AdvantageKit IO.
  private final TurretAimManager turretAimManager = new TurretAimManager(drive::getPose);

  // Acessors
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

  public Turret getTurretRight() {
    return turretRight;
  }

  public TurretAimManager getTurretAimManager() {
    return turretAimManager;
  }

  // Autonomous commands
  private final frc.robot.Auton.AutonCommandBase autonCommands;

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(1);
  private final CommandXboxController operator = new CommandXboxController(0);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Initialize autonomous commands
    autonCommands = new frc.robot.Auton.AutonCommandBase(drive);

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Automatically register all autonomous commands from AutonCommandBase
    autonCommands.registerAllCommands(autoChooser);

    // Also manually add them as backup (for testing)
    autoChooser.addOption("Forward 2m (Manual)", autonCommands.forward2m());
    autoChooser.addOption("Example Auto (Manual)", autonCommands.exampleAuto());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // #region Dashboard Buttons
    SmartDashboard.putData(
        "inPitSwitch",
        Commands.parallel(
            new InstantCommand(() -> Constants.overrideState = Constants.Override.OFF),
            new InstantCommand(() -> Constants.inPit = !Constants.inPit),
            intakePivot.setCoast(),
            intakeRollers.setCoast(),
            agitatorLeft.setCoast(),
            agitatorRight.setCoast(),
            verticalFeedLeft.setCoast(),
            verticalFeedRight.setCoast(),
            hoodLeft.setCoast(),
            hoodRight.setCoast(),
            turretLeft.setCoast(),
            turretRight.setCoast(),
            flywheelLeft.setCoast(),
            flywheelRight.setCoast()));

    SmartDashboard.putData("intake", intakeRollers.intakeCommand());
    SmartDashboard.putData("outtake", intakeRollers.outtakeCommand());
    SmartDashboard.putData(
        "shoot",
        Commands.parallel(
            flywheelLeft.shootCommand(),
            flywheelRight.shootCommand(),
            verticalFeedLeft.verticalFeedIntakeCommand(),
            verticalFeedRight.verticalFeedIntakeCommand(),
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
        intakePivot.setDegreesCommand(Constants.IntakeConstants.kIntakePivotDeployDegrees));
    SmartDashboard.putData(
        "retract intake",
        intakePivot.setDegreesCommand(Constants.IntakeConstants.kIntakePivotStowedDegrees));
    SmartDashboard.putData(
        "floors on",
        Commands.parallel(agitatorLeft.snowblowCommand(), agitatorRight.snowblowCommand()));
    SmartDashboard.putData(
        "floors off", Commands.parallel(agitatorLeft.offCommand(), agitatorRight.offCommand()));
    SmartDashboard.putData(
        "flywheel on",
        Commands.parallel(flywheelLeft.shootCommand(), flywheelRight.shootCommand()));
    SmartDashboard.putData(
        "flywheel off", Commands.parallel(flywheelLeft.offCommand(), flywheelRight.offCommand()));
    SmartDashboard.putData(
        "Move Hood 5 degrees",
        Commands.parallel(
            hoodLeft.setDegreesCommand(hoodLeft.getCurrentPosition() + 5.0),
            hoodRight.setDegreesCommand(hoodRight.getCurrentPosition() + 5.0)));

    SmartDashboard.putData(
        "Change Hub Active",
        new InstantCommand(() -> Constants.hubOverride = !Constants.hubOverride));
    // #endregion

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive (same in both modes)
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // #region Driver Controls
    // --- DRIVER BINDINGS -----------------------------------------------------------------------

    // A = pit: snap robot to 0° | normal = lock to 0°
    driver
        .y()
        .whileTrue(
            Commands.either(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> Rotation2d.fromDegrees(0.0)),
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> Rotation2d.kZero),
                () -> Constants.inPit));

    // B: pit = snap robot to 90° | normal = zero gyro
    driver
        .x()
        .whileTrue(
            Commands.either(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> Rotation2d.fromDegrees(90.0)),
                Commands.runOnce(
                        () ->
                            drive.setPose(
                                new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                        drive)
                    .ignoringDisable(true),
                () -> Constants.inPit));

    // X: = pit: snap robot to 180° | normal: set pose to Red Hub
    driver
        .a()
        .whileTrue(
            Commands.either(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> Rotation2d.fromDegrees(180.0)),
                Commands.runOnce(
                        () ->
                            drive.setPose(
                                new Pose2d(new Translation2d(10.942, 4.042), Rotation2d.kZero)),
                        drive)
                    .ignoringDisable(true),
                () -> Constants.inPit));

    // Y: pit = snap robot to 270°
    driver
        .b()
        .whileTrue(
            Commands.either(
                DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -driver.getLeftY(),
                    () -> -driver.getLeftX(),
                    () -> Rotation2d.fromDegrees(270.0)),
                Commands.none(),
                () -> Constants.inPit));

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

    // right bumper = pit: intake | normal: none
    driver
        .rightBumper()
        .toggleOnTrue(
            Commands.either(intakeRollers.intakeCommand(), Commands.none(), () -> Constants.inPit));

    // left bumper = pit: outtake | normal: none
    driver
        .leftBumper()
        .toggleOnTrue(
            Commands.either(
                intakeRollers.outtakeCommand(), Commands.none(), () -> Constants.inPit));

    // Driver pit controls: right trigger = shoot (both vertical feeders)
    driver
        .rightTrigger()
        .toggleOnTrue(
            Commands.either(
                Commands.parallel(
                    verticalFeedLeft.verticalFeedIntakeCommand(),
                    verticalFeedRight.verticalFeedIntakeCommand()),
                Commands.none(),
                () -> Constants.inPit));

    driver
        .leftTrigger()
        .toggleOnTrue(
            Commands.either(
                Commands.parallel(verticalFeedLeft.offCommand(), verticalFeedRight.offCommand()),
                Commands.none(),
                () -> Constants.inPit));
    // #endregion

    // #region Operator Controls
    // ----------------------------------------------------------------------------------------------------

    if (turretLeft != null) {

      // Pit: left joystick angle maps directly to left turret angle (±220°).
      // Only updates when stick is pushed past deadband magnitude.
      turretLeft.setDefaultCommand(
          turretLeft.dutyCycleCommand(
              () -> {
                if (!Constants.inPit) return 0.0;
                double x = operator.getLeftX();
                double y = operator.getLeftY();
                if (Math.sqrt(x * x + y * y) < 0.5) return 0.0;
                double angleDeg = Math.toDegrees(Math.atan2(x, -y)); // 0° = stick up, +CW
                double clamped =
                    Math.max(
                        Constants.ScorerConstants.kTurretMinPositionUnits,
                        Math.min(Constants.ScorerConstants.kTurretMaxPositionUnits, angleDeg));
                turretLeft.setDegreesCommand(clamped).schedule();
                return 0.0;
              }));
    }

    if (turretRight != null) {
      // Pit: right joystick angle maps directly to right turret angle (±220°).
      turretRight.setDefaultCommand(
          turretRight.dutyCycleCommand(
              () -> {
                if (!Constants.inPit) return 0.0;
                double x = operator.getRightX();
                double y = operator.getRightY();
                if (Math.sqrt(x * x + y * y) < 0.5) return 0.0;
                double angleDeg = Math.toDegrees(Math.atan2(x, -y));
                double clamped =
                    Math.max(
                        Constants.ScorerConstants.kTurretMinPositionUnits,
                        Math.min(Constants.ScorerConstants.kTurretMaxPositionUnits, angleDeg));
                turretRight.setDegreesCommand(clamped).schedule();
                return 0.0;
              }));
    }

    if (agitatorRight != null && agitatorLeft != null) {
      // Y = pit: shooter rollers on | normal: snowblow state
      operator
          .y()
          .toggleOnTrue(
              Commands.either(
                  Commands.parallel(flywheelLeft.shootCommand(), flywheelRight.shootCommand()),
                  new InstantCommand(() -> Constants.currentState = Constants.BotState.SNOWBLOW),
                  () -> Constants.inPit));
    }

    // X = pit: shooter rollers off | normal: defence state
    operator
        .x()
        .toggleOnTrue(
            Commands.either(
                Commands.parallel(flywheelLeft.offCommand(), flywheelRight.offCommand()),
                new InstantCommand(() -> Constants.currentState = Constants.BotState.DEFENCE),
                () -> Constants.inPit));

    // B = pit: floors on | normal: defence state
    operator
        .b()
        .toggleOnTrue(
            Commands.either(
                Commands.parallel(agitatorLeft.snowblowCommand(), agitatorRight.snowblowCommand()),
                new InstantCommand(() -> Constants.currentState = Constants.BotState.DEFENCE),
                () -> Constants.inPit));

    // A = pit: floors off | normal: collect state
    operator
        .a()
        .toggleOnTrue(
            Commands.either(
                Commands.parallel(agitatorLeft.offCommand(), agitatorRight.offCommand()),
                new InstantCommand(() -> Constants.currentState = Constants.BotState.COLLECT),
                () -> Constants.inPit));

    // Right Trigger = normal: cycle override state | pit: vertical rollers on
    operator
        .rightTrigger()
        .onTrue(
            Commands.either(
                // Pit mode: vertical on
                Commands.parallel(
                    verticalFeedLeft.verticalFeedIntakeCommand(),
                    verticalFeedRight.verticalFeedIntakeCommand()),
                // Normal mode: cycle override state
                Commands.runOnce(
                    () -> {
                      switch (Constants.overrideState) {
                        case OFF:
                          Constants.overrideState = Constants.Override.ON;
                          break;
                        case ON:
                          Constants.overrideState = Constants.Override.COLLECT;
                          break;
                        case COLLECT:
                          Constants.overrideState = Constants.Override.DEFENCE;
                          break;
                        case DEFENCE:
                          Constants.overrideState = Constants.Override.ON;
                          break;
                        default:
                          Constants.overrideState = Constants.Override.OFF;
                          break;
                      }
                    }),
                () -> Constants.inPit));

    // Left Trigger = pit: vertical rollers on | normal: none
    if (verticalFeedRight != null && verticalFeedLeft != null) {
      operator
          .leftTrigger()
          .toggleOnTrue(
              Commands.either(
                  Commands.parallel(verticalFeedLeft.offCommand(), verticalFeedRight.offCommand()),
                  Commands.none(),
                  () -> Constants.inPit));
    }

    // Right bumper = both: deploy intake
    operator.rightBumper().onTrue(intakePivot.deployCommand());

    // Left bumper = pit: retract intake | normal: auto-aim
    operator
        .leftBumper()
        .onTrue(
            Commands.either(
                // Pit: retract intake
                intakePivot.retractCommand(),
                // Normal: auto-aim turrets and hoods
                Commands.parallel(
                    hoodLeft.setDegreesCommand(turretAimManager.getLeftHoodAngleDeg()),
                    hoodRight.setDegreesCommand(turretAimManager.getRightHoodAngleDeg()),
                    turretLeft.setDegreesCommand(turretAimManager.getLeftTurretAngleDeg()),
                    turretRight.setDegreesCommand(turretAimManager.getRightTurretAngleDeg())),
                () -> Constants.inPit));

    if (flywheelLeft != null && hoodLeft != null) {
      // D-Pad Up/Down: normal = flywheel + hood | pit = move both hoods to preset
      // degrees
      operator
          .povUp()
          .onTrue(
              Commands.either(
                  // Pit: hoods to 35 deg
                  Commands.parallel(
                      hoodLeft.setDegreesCommand(Constants.ScorerConstants.kHoodMaxDegrees),
                      hoodRight.setDegreesCommand(Constants.ScorerConstants.kHoodMaxDegrees)),
                  // Normal: flywheel on + hood up
                  Commands.parallel(
                      flywheelLeft.shootCommand(),
                      hoodLeft.setDegreesCommand(Constants.ScorerConstants.kHoodMaxDegrees)),
                  () -> Constants.inPit));
      operator
          .povDown()
          .onTrue(
              Commands.either(
                  // Pit: hoods to 10 deg (stowed)
                  Commands.parallel(
                      hoodLeft.setDegreesCommand(Constants.ScorerConstants.kHoodMinDegrees),
                      hoodRight.setDegreesCommand(Constants.ScorerConstants.kHoodMinDegrees)),
                  // Normal: flywheel off + hood stow
                  Commands.parallel(
                      flywheelLeft.offCommand(),
                      hoodLeft.setDegreesCommand(Constants.ScorerConstants.kHoodMinDegrees),
                      hoodRight.setDegreesCommand(Constants.ScorerConstants.kHoodMinDegrees)),
                  () -> Constants.inPit));
      // Pit: D-Pad Right = hoods to 25 deg, D-Pad Left = hoods to 20 deg
      operator
          .povRight()
          .onTrue(
              Commands.either(
                  Commands.parallel(
                      hoodLeft.setDegreesCommand(25.0), hoodRight.setDegreesCommand(25.0)),
                  Commands.none(),
                  () -> Constants.inPit));
      operator
          .povLeft()
          .onTrue(
              Commands.either(
                  Commands.parallel(
                      hoodLeft.setDegreesCommand(20.0), hoodRight.setDegreesCommand(20.0)),
                  Commands.none(),
                  () -> Constants.inPit));
    }
    // #endregion
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
