// Copyright (c) 2026 FRC Team 3310
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file.

package frc.robot.Auton;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.lang.reflect.Method;
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
   * Command to drive forward 2 meters.
   *
   * @return Command that follows the forward2m path
   */
  public Command forward2m() {
    if (autoPaths.forward2m == null) {
      System.err.println("forward2m path not loaded!");
      return drive.run(() -> {}).withName("Forward2m (Not Loaded)");
    }
    return new FollowPathCommand(drive, autoPaths.forward2m).withName("Forward 2m");
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
