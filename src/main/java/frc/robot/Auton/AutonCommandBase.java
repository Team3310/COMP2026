package frc.robot.Auton;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import java.lang.reflect.Method;
import java.util.Set;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * Container class for all autonomous commands. Automatically discovers and registers all public
 * Command methods for easy access.
 */
public class AutonCommandBase {
  private final Drive drive;
  private final paths autoPaths;

  /**
   * Creates a new AutonCommandBase.
   *
   * @param drive The drive subsystem
   */
  public AutonCommandBase(Drive drive) {
    this.drive = drive;
    this.autoPaths = new paths();
  }

  /**
   * Automatically adds all autonomous commands to the chooser. Scans for all public methods that
   * return Command and adds them to the dashboard chooser.
   *
   * @param chooser The dashboard chooser to add commands to
   */
  public void registerAllCommands(LoggedDashboardChooser<Command> chooser) {
    System.out.println("=== Registering Autonomous Commands ===");
    int registeredCount = 0;

    // Get all methods from this class
    for (Method method : this.getClass().getDeclaredMethods()) {
      // Check if method returns Command and takes no parameters
      if (method.getReturnType() == Command.class && method.getParameterCount() == 0) {
        try {
          // Convert method name to readable format (camelCase -> Camel Case)
          String commandName = formatMethodName(method.getName());

          // Get the command by calling the method
          Command command = (Command) method.invoke(this);

          // Add to chooser
          chooser.addOption(commandName, command);
          registeredCount++;
          System.out.println("  ✓ Registered: " + commandName + " (" + method.getName() + ")");
        } catch (Exception e) {
          System.err.println("  ✗ Failed to register: " + method.getName());
          e.printStackTrace();
        }
      }
    }
    System.out.println("=== Total Commands Registered: " + registeredCount + " ===");
  }

  /**
   * Converts camelCase method name to Title Case display name.
   *
   * @param methodName The method name in camelCase
   * @return The formatted display name
   */
  private String formatMethodName(String methodName) {
    // Insert spaces before capital letters and capitalize first letter
    return methodName.replaceAll("([A-Z])", " $1").trim();
  }

  // -------------------- Autonomous Commands --------------------

  /**
   * Command to drive forward 2 meters from current position. Uses on-the-fly path generation to
   * move relative to current pose.
   *
   * @return Command that drives 2m forward from current position
   */
  public Command forward2m() {
    return Commands.defer(
            () -> {
              // Get current pose when command starts
              Pose2d currentPose = drive.getPose();

              // Calculate target pose 2 meters forward in current direction
              Translation2d currentTranslation = currentPose.getTranslation();
              var rotation = currentPose.getRotation();

              // Move 2m in the direction the robot is facing
              Translation2d offset = new Translation2d(2.0, rotation);
              Translation2d targetTranslation = currentTranslation.plus(offset);

              Pose2d targetPose = new Pose2d(targetTranslation, rotation);

              // Generate path on-the-fly from current to target
              PathPlannerPath path =
                  new PathPlannerPath(
                      PathPlannerPath.waypointsFromPoses(currentPose, targetPose),
                      new PathConstraints(
                          1.0, // maxVelocityMps
                          1.0, // maxAccelerationMpsSq
                          2 * Math.PI, // maxAngularVelocityRps
                          4 * Math.PI // maxAngularAccelerationRpsSq
                          ),
                      null, // idealStartingState - null means it will be calculated
                      new GoalEndState(0.0, rotation) // end velocity, end rotation
                      );

              // Prevent path from flipping
              path.preventFlipping = true;

              // Follow the generated path
              return AutoBuilder.followPath(path);
            },
            Set.of(drive))
        .withName("Forward 2m Relative");
  }

  /**
   * Example multi-step autonomous routine.
   *
   * @return Command sequence
   */
  public Command exampleAuto() {
    return forward2m().withName("Example Auto");
  }

  // Add more autonomous commands here - they'll be auto-registered!
  // Just make them public and return Command with no parameters:
  //
  // public Command threePieceAuto() {
  //   return Commands.sequence(...);
  // }
  //
  // public Command scoreAndLeave() {
  //   return Commands.sequence(...);
  // }
}
