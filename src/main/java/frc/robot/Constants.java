// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  // ===========================================================================
  // ROBOT SELECTION — change this ONE line before deploying.
  //   Bot.COMP     -> competition robot  (TunerConstantsComp)
  //   Bot.PRACTICE -> practice robot     (TunerConstantsPractice)
  // ===========================================================================
  public static final Bot currentBot = Bot.COMP;

  public static enum Bot {
    /** Competition robot */
    COMP,
    /** Practice robot */
    PRACTICE
  }

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  // Global motor voltage limit applied to all TalonFX motors
  public static final double kMotorPeakVoltage = 12.0;

  // Global log rate-limiting — every Nth 20 ms cycle.
  // All subsystems share this interval but use different counter offsets
  // so their Logger.recordOutput calls are staggered across cycles.
  public static final int kLogInterval = 10; // ~5 Hz at 50 Hz loop

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // #region Simulation Physics
  public static final class SimPhysicsConstants {
    // ---- Translational inertia (forward / backward) ----
    // Time constant in seconds. Higher = slower acceleration.
    // ~0.15 s approximates a 75 kg robot with ~4 Kraken X60s on geared swerve.
    // public static double kTranslationalTauSeconds = 0.15;
    public static double kTranslationalTauSeconds = 0.12;

    // ---- Lateral (strafe) inertia ----
    // Typically similar to translational, but can be tuned separately to model
    // higher lateral scrub / lower traction.
    // public static double kLateralTauSeconds = 0.18;
    public static double kLateralTauSeconds = 0.12;

    // ---- Rotational inertia ----
    // Time constant for angular velocity. Higher = harder to start/stop spinning.
    // ~0.12 s is reasonable for a compact swerve with MOI ~6.9 kg·m².
    public static double kRotationalTauSeconds = 0.12;

    // ---- Soft speed caps ----
    // These define the *effective* top speed the sim robot can reach.
    // Instead of a hard wall, a drag model smoothly reduces acceleration as
    // speed approaches the cap.  The robot asymptotically approaches but never
    // exceeds the cap.  Set these independently of the tau values above.
    //
    // Translational soft cap (m/s).  The real kSpeedAt12Volts is the motor's
    // theoretical max; this should be ≤ that value to model traction limits,
    // carpet drag, etc.
    public static double kTranslationalSoftCapMps = 7.0;

    // Rotational soft cap (rad/s).  Real max ≈ kSpeedAt12Volts / driveBaseRadius.
    // Lower this to model realistic turn-rate limits.
    public static double kRotationalSoftCapRadPerSec = 8.0;

    // ---- Drag curve exponent ----
    // Controls the shape of the rolloff near the soft cap.
    //   1.0  = linear rolloff   (gentle, starts limiting early)
    //   2.0  = quadratic        (moderate — good default)
    //   3.0+ = sharper knee     (feels fast until near the cap, then drops off)
    // The drag factor is:  max(0, 1 - (speed / softCap) ^ exponent)
    public static double kDragExponent = 2.0;

    // ---- Simulated gyro noise (degrees per second, 1σ) ----
    // Adds a small random walk to the sim gyro to mimic real sensor noise.
    public static double kGyroNoiseDegPerSec = 0.05;
  }

  // #endregion

  // #region Drive Command Tuning
  // Constants used by frc.robot.commands.DriveCommands.
  // Usage references:
  //   - DriveCommands.getLinearVelocityFromJoysticks()
  //   - DriveCommands.joystickDrive()
  //   - DriveCommands.joystickDriveAtAngle()
  //   - DriveCommands.feedforwardCharacterization()
  //   - DriveCommands.wheelRadiusCharacterization()
  public static final class DriveCommandConstants {
    // Joystick deadband applied to translation magnitude in
    // getLinearVelocityFromJoysticks().
    public static final double kJoystickDeadband = 0.1;

    // Joystick deadband applied to rotational command shaping in joystickDrive().
    public static final double kRotationCommandDeadband = kJoystickDeadband;

    // Larger neutral threshold used to decide when heading hold should engage in joystickDrive().
    public static final double kHoldStickNeutralDeadband = 0.25;

    // Maximum measured yaw rate (rad/s) allowed before heading hold can engage in joystickDrive().
    // If robot rotates faster than this, hold setpoint tracks current heading instead of locking.
    public static final double kHoldEngageMaxRateRadPerSec = Units.degreesToRadians(15.0);

    // PID gains used for heading hold in joystickDrive() and angle control in
    // joystickDriveAtAngle().
    public static final double kAngleHoldKp = 7.0;
    // was 5.0. PDC put it to 3 on 3-18 before practice
    public static final double kAngleHoldKd = 0.0;

    // Trapezoid profile limits used only in joystickDriveAtAngle().
    public static final double kAngleProfileMaxVelocityRadPerSec = 12.0;
    public static final double kAngleProfileMaxAccelerationRadPerSec2 = 20.0;

    // Feedforward characterization timing/ramp values used only in
    // feedforwardCharacterization().
    public static final double kFfCharacterizationStartDelaySec = 2.0;
    public static final double kFfCharacterizationRampRate = 0.1; // Volts per second

    // Wheel-radius characterization limits used only in wheelRadiusCharacterization().
    public static final double kWheelRadiusCharacterizationMaxVelocity = 0.45; // Rad/s
    public static final double kWheelRadiusCharacterizationRampRate = 0.05; // Rad/s^2

    // Driver face-button snap targets (degrees).
    // RobotContainer mappings:
    //   - Y -> kDriverSnapAngleYDeg
    //   - X -> kDriverSnapAngleXDeg
    //   - A -> kDriverSnapAngleADeg
    //   - B -> kDriverSnapAngleBDeg
    public static final double kDriverSnapAngleYDeg = 0.0;
    public static final double kDriverSnapAngleXDeg = 90.0;
    public static final double kDriverSnapAngleADeg = 180.0;
    public static final double kDriverSnapAngleBDeg = 270.0;

    // Normal teleop drive profile. These are driver-facing chassis limits, not
    // the drivetrain's physical module-speed ceiling.
    public static double kNormalMaxLinearSpeedMps = 4.0;
    public static double kNormalMaxAngularSpeedRadPerSec = 9.0;
    public static double kNormalMaxLinearAccelMetersPerSec2 = 9.0;
    public static double kNormalMaxAngularAccelRadPerSec2 = 30.0;
    public static double kNormalMaxLinearDecelMetersPerSec2 = 40.0;
    public static double kNormalMaxAngularDecelRadPerSec2 = 60.0;

    // Driver right-trigger alternate center of rotation, robot-relative.
    public static final double kDriverAltCenterOfRotationXMeters = Units.inchesToMeters(23.125);
    public static final double kDriverAltCenterOfRotationYMeters = 0.0;
  }

  // #endregion
}
