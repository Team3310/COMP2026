package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
import frc.lib.util.FieldConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.agitator.Agitator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.IntakePivot;
import frc.robot.subsystems.intake.IntakeRollers;
import frc.robot.subsystems.scorer.flywheel.Flywheel;
import frc.robot.subsystems.scorer.hood.Hood;
import frc.robot.subsystems.scorer.turret.Turret;
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
          new GyroIO() {},
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

    SmartDashboard.putData(
        "Change Hub Active",
        new InstantCommand(() -> Constants.hubOverride = !Constants.hubOverride));

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
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));

    // Lock to 0° when A button is held
    driver
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    driver
        .x()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                FieldConstants.StartingPosition.REDHUB.getTranslation(),
                                Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Reset gyro to 0° when B button is pressed
    driver
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Zero gyro with Start button (same as B, easier to reach mid-match)
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    if (agitatorRight != null && agitatorLeft != null) {
      // Y button = toggle agitator rollers on/off (closed loop PID)
      operator
          .y()
          .toggleOnTrue(
              Commands.parallel(agitatorRight.snowblowCommand(), agitatorLeft.snowblowCommand()));
    }

    if (verticalFeedRight != null && verticalFeedLeft != null) {

      // Left Trigger = toggle vertical feed roller on/off (closed loop PID)
      operator
          .leftTrigger()
          .toggleOnTrue(
              Commands.parallel(
                  verticalFeedLeft.verticalFeedIntakeCommand(),
                  verticalFeedRight.verticalFeedIntakeCommand()));
    }

    operator
        .rightTrigger()
        .onTrue(
            Commands.runOnce(
                () -> {
                  switch (Constants.overrideState) {
                    case FALSE:
                      Constants.overrideState = Constants.Override.COLLECT;
                      break;
                    case COLLECT:
                      Constants.overrideState = Constants.Override.DEFENCE;
                      break;
                    case DEFENCE:
                      Constants.overrideState = Constants.Override.FALSE;
                      break;
                    default:
                      Constants.overrideState = Constants.Override.FALSE;
                      break;
                  }
                }));

    if (intakeRollers != null && intakePivot != null) {
      // Right bumper = toggle intake rollers on/off
      operator
          .rightBumper()
          .toggleOnTrue(
              Commands.parallel(intakeRollers.deployCommand(), intakePivot.setDegreesCommand(145.0))
                  .andThen(intakeRollers.intakeCommand()));
      // Left bumper = toggle outtake rollers on/off
      operator
          .leftBumper()
          .toggleOnTrue(
              Commands.parallel(intakeRollers.offCommand(), intakePivot.setDegreesCommand(0.0)));
    }

    // if (intakePivot != null) {
    // operator.povUp().onTrue(intakePivot.setDegreesCommand(0.0));
    // operator.povDown().onTrue(intakePivot.setDegreesCommand(145.0));
    // }

    // if (hoodLeft != null) {
    // operator.povUp().onTrue(hoodLeft.setDegreesCommand(35.0));
    // operator.povDown().onTrue(hoodLeft.setDegreesCommand(10.0));
    // }

    if (turretLeft != null) {
      operator.povRight().onTrue(turretLeft.setDegreesCommand(-45.0));
      operator.povLeft().onTrue(turretLeft.setDegreesCommand(0.0));
    }

    if (flywheelLeft != null && hoodLeft != null) {
      operator
          .povUp()
          .onTrue(
              Commands.parallel(
                  flywheelLeft.forwardCommand(),
                  hoodLeft.setDegreesCommand(Constants.ScorerConstants.kHoodMaxPositionUnits)));
      operator
          .povDown()
          .onTrue(
              Commands.parallel(
                  flywheelLeft.offCommand(),
                  hoodLeft.setDegreesCommand(Constants.ScorerConstants.kHoodStowedPosition)));
    }
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
