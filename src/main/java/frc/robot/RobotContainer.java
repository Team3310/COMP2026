package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  // Subsystem Instances
  private final Drive drive = buildDriveSystem();

  // Controllers
  private final CommandXboxController driver = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  public Drive getDrive() {
    return drive;
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  private RobotContainer() {
    configureAutoChooser();
    configureButtonBindings();
  }

  private void configureAutoChooser() {
    autoChooser.addDefaultOption("None", Commands.none());
    autoChooser.addOption("Forward 2m (Path)", buildFollowPathAuto("forward2m"));

    // Drivetrain characterization
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
  }

  /** Resets odometry to the path's start pose (alliance-flipped as needed), then follows it. */
  private Command buildFollowPathAuto(String pathName) {
    final PathPlannerPath path;
    try {
      path = PathPlannerPath.fromPathFile(pathName);
    } catch (Exception e) {
      DriverStation.reportError("Failed to load path '" + pathName + "': " + e.getMessage(), false);
      return Commands.none();
    }

    return Commands.runOnce(
            () -> {
              PathPlannerPath startPath = AutoBuilder.shouldFlip() ? path.flipPath() : path;
              startPath.getStartingHolonomicPose().ifPresent(drive::setPose);
            },
            drive)
        .andThen(AutoBuilder.followPath(path));
  }

  private Command buildSnapCommand(double targetAngleDeg) {
    return DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> getDriverPerspectiveSnapAngle(targetAngleDeg),
            this::getDriverCenterOfRotation)
        .until(
            () ->
                Math.abs(driver.getRightX())
                    > Constants.DriveCommandConstants.kRotationCommandDeadband);
  }

  private Translation2d getDriverCenterOfRotation() {
    return driver.getHID().getRightTriggerAxis() > 0.5
        ? new Translation2d(
            Constants.DriveCommandConstants.kDriverAltCenterOfRotationXMeters,
            Constants.DriveCommandConstants.kDriverAltCenterOfRotationYMeters)
        : Translation2d.kZero;
  }

  private Rotation2d getDriverPerspectiveSnapAngle(double blueFrameAngleDeg) {
    Rotation2d targetAngle = Rotation2d.fromDegrees(blueFrameAngleDeg);
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
        ? targetAngle.plus(Rotation2d.kPi)
        : targetAngle;
  }

  private void configureButtonBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX(),
            this::getDriverCenterOfRotation));

    // Face buttons: snap to angle and hold until the driver pushes right-stick X
    // outside the rotation deadband.
    driver.y().whileTrue(buildSnapCommand(Constants.DriveCommandConstants.kDriverSnapAngleYDeg));
    driver.x().whileTrue(buildSnapCommand(Constants.DriveCommandConstants.kDriverSnapAngleXDeg));
    driver.a().whileTrue(buildSnapCommand(Constants.DriveCommandConstants.kDriverSnapAngleADeg));
    driver.b().whileTrue(buildSnapCommand(Constants.DriveCommandConstants.kDriverSnapAngleBDeg));

    // Start: zero gyro
    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Right trigger: alternate center of rotation (handled in getDriverCenterOfRotation)
  }
}
