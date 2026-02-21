// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
// I am the coolest person on earth, next to john doe
package frc.robot;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotBase;
import frc.lib.drivers.CANDeviceId;
import frc.lib.subsystems.ServoMotorSubsystemConfig;
import frc.lib.subsystems.ServoMotorSubsystemWithCanCoderConfig;
import frc.robot.generated.TunerConstants;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;
  public static final Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);

  public static boolean deploying = false;
  public static boolean retracting = false;
  public static boolean activeHub = true;
  public static boolean hubOverride = false; // true = manually forced OFF by SmartDashboard button
  public static BotState currentState = BotState.COLLECT;
  public static Override overrideState = Override.FALSE;

  public static final String kPracticeBotMacAddress = "00:80:2F:33:BF:BB";
  public static boolean kIsPracticeBot = hasMacAddress(kPracticeBotMacAddress);

  // Set this to select which robot's tuner constants to use
  public static final Bot currentBot = Bot.BRAVO;

  // Field dimensions for 2026 Reefscape
  public static final double kFieldLengthMeters = 16.54;
  public static final double kFieldWidthMeters = 8.21;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static enum Bot {
    /** Software robot (default configuration) */
    SOFTWARE,
    /** Bravo robot */
    BRAVO
    // Add more robot variants here as needed, e.g.:
    // COMPETITION,
    // PRACTICE
  }

  public static enum BotState {
    SNOWBLOW,
    COLLECT,
    DEFENCE,
    DEPLOY,
    RETRACT
  }

  public static enum Override {
    FALSE,
    COLLECT,
    DEFENCE
  }

  public static final ClosedLoopRampsConfigs makeDefaultClosedLoopRampConfig() {
    return new ClosedLoopRampsConfigs()
        .withDutyCycleClosedLoopRampPeriod(0.02)
        .withTorqueClosedLoopRampPeriod(0.02)
        .withVoltageClosedLoopRampPeriod(0.02);
  }

  public static final OpenLoopRampsConfigs makeDefaultOpenLoopRampConfig() {
    return new OpenLoopRampsConfigs()
        .withDutyCycleOpenLoopRampPeriod(0.02)
        .withTorqueOpenLoopRampPeriod(0.02)
        .withVoltageOpenLoopRampPeriod(0.02);
  }

  // #region Vision
  // -------------------------------------------------------------------------
  // MegaTag 2 Vision Constants — 3× Limelight 4 cameras
  // Camera positions are from the Practice Robot Software Design Sheet.
  // Coordinate system: LL Robot-Space — forward(+X), side(+Y left), up(+Z).
  // All linear values converted from inches to meters.
  // -------------------------------------------------------------------------
  public static final class VisionConstants {
    // Camera hostnames (must match Limelight web UI / network config)
    public static final String kLimelightRear = "limelight-rear";
    public static final String kLimelightRight = "limelight-right";
    public static final String kLimelightLeft = "limelight-left";

    public static final String[] kCameraNames = {kLimelightRear, kLimelightRight, kLimelightLeft};

    // ---- Camera #1  (Rear-facing) ----
    // SDS: X = -1.098 in, Y = 0 in, Z = 20.338 in
    //      Zrot = 180°, Yrot(pitch) = 20°, Xrot(roll) = 0° (TBD treated as 0)
    public static final double kRearForwardM = Units.inchesToMeters(-1.098);
    public static final double kRearSideM = Units.inchesToMeters(0.0);
    public static final double kRearUpM = Units.inchesToMeters(20.338);
    public static final double kRearRollDeg = 0.0;
    public static final double kRearPitchDeg = 20.0;
    public static final double kRearYawDeg = 180.0;

    // ---- Camera #2  (Right-side, mounted upside-down) ----
    // SDS: X = -3.132 in, Y = -13.179 in, Z = 13.558 in
    //      Zrot = -90°, Yrot = 180° (upside-down), Xrot = 0°
    public static final double kRightForwardM = Units.inchesToMeters(-3.132);
    public static final double kRightSideM = Units.inchesToMeters(-13.179);
    public static final double kRightUpM = Units.inchesToMeters(13.558);
    public static final double kRightRollDeg = 180.0; // upside-down
    public static final double kRightPitchDeg = 0.0;
    public static final double kRightYawDeg = -90.0;

    // ---- Camera #3  (Left-side) ----
    // SDS: X = -3.312 in, Y = 13.179 in, Z = 13.558 in
    //      Zrot = 90°, Yrot = 0°, Xrot = 0°
    public static final double kLeftForwardM = Units.inchesToMeters(-3.312);
    public static final double kLeftSideM = Units.inchesToMeters(13.179);
    public static final double kLeftUpM = Units.inchesToMeters(13.558);
    public static final double kLeftRollDeg = 0.0;
    public static final double kLeftPitchDeg = 0.0;
    public static final double kLeftYawDeg = 90.0;

    // Camera poses packed as {forward, side, up, roll, pitch, yaw} for
    // LimelightHelpers.setCameraPose_RobotSpace()
    public static final double[][] kCameraPoses = {
      {kRearForwardM, kRearSideM, kRearUpM, kRearRollDeg, kRearPitchDeg, kRearYawDeg},
      {kRightForwardM, kRightSideM, kRightUpM, kRightRollDeg, kRightPitchDeg, kRightYawDeg},
      {kLeftForwardM, kLeftSideM, kLeftUpM, kLeftRollDeg, kLeftPitchDeg, kLeftYawDeg}
    };

    // ---- Filtering thresholds ----
    // Maximum angular velocity (deg/s) before we reject vision updates.
    // Fast rotation causes motion-blur → bad detections.
    public static final double kMaxAngularVelocityDegPerSec = 720.0;

    // Minimum average tag area (% of image) required to trust a single-tag result
    public static final double kMinTagAreaForSingleTag = 0.1;

    // Maximum allowed distance from prior pose before rejecting (meters)
    public static final double kMaxPoseJumpMeters = 1.5;

    // ---- Limelight NT std-dev array ----
    // The Limelight publishes a 12-element "stddevs" array on NetworkTables:
    //   [MT1x, MT1y, MT1z, MT1roll, MT1pitch, MT1yaw,
    //    MT2x, MT2y, MT2z, MT2roll, MT2pitch, MT2yaw]
    // We only use the MegaTag 2 entries (indices 6–11).
    public static final int kExpectedStdDevArrayLength = 12;
    public static final int kMT2XStdDevIndex = 6;
    public static final int kMT2YStdDevIndex = 7;

    // Theta std dev — set very high because MegaTag 2 uses gyro for rotation.
    // We ignore the LL-reported yaw std dev and always override with this value.
    public static final double kThetaStdDev = 999999.0;
  }
  // #endregion

  // #region Scorer Subsystems
  public static final class ScorerConstants {
    public static final double kShootRPM = 1000.0;
    public static final double kReverseRPM = -1000.0;

    public static final double kHoodStowedPosition = 10.0; // degrees
    public static final double kHoodMaxPositionUnits = 35.0; // degrees
    public static final double kHoodMinPositionUnits = 10.0; // degrees
    public static final double kHoodUnitToRotorRatio =
        (10.0 / 44.0) * (18.0 / 294.0) * 360.0; // convert rotations to degrees
    public static final double kHoodMomentOfInertia = 0.01; // kg*m^2 (estimate for tuning)

    public static final double kTurretStowedPosition = 0.0; // degrees
    public static final double kTurretMaxPositionUnits = 90.0; // degrees
    public static final double kTurretMinPositionUnits = -90.0; // degrees
    public static final double kTurretUnitToRotorRatio =
        (11.0 / 32.0) * (14.0 / 220.0) * 360.0; // convert rotations to degrees
    public static final double kTurretMomentOfInertia = 0.01; // kg*m^2 (estimate for tuning)
  }

  public static final ServoMotorSubsystemConfig kLeftHoodConfig = new ServoMotorSubsystemConfig();

  static {
    kLeftHoodConfig.name = "Left Hood";
    kLeftHoodConfig.talonCANID = new CANDeviceId(23, TunerConstants.kCANBus1.getName());

    kLeftHoodConfig.unitToRotorRatio = ScorerConstants.kHoodUnitToRotorRatio;
    kLeftHoodConfig.kMaxPositionUnits = ScorerConstants.kHoodMaxPositionUnits;
    kLeftHoodConfig.kMinPositionUnits = ScorerConstants.kHoodMinPositionUnits;
    kLeftHoodConfig.momentOfInertia = ScorerConstants.kHoodMomentOfInertia;

    kLeftHoodConfig.fxConfig.Slot0.kP = 2.0;
    kLeftHoodConfig.fxConfig.Slot0.kD = 0.0;
    kLeftHoodConfig.fxConfig.Slot0.kV = 0.2;
    kLeftHoodConfig.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 80.0;
    kLeftHoodConfig.fxConfig.MotionMagic.MotionMagicAcceleration = 300.0;

    kLeftHoodConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    kLeftHoodConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        kLeftHoodConfig.kMaxPositionUnits / kLeftHoodConfig.unitToRotorRatio;
    kLeftHoodConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    kLeftHoodConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        kLeftHoodConfig.kMinPositionUnits / kLeftHoodConfig.unitToRotorRatio;

    kLeftHoodConfig.fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kLeftHoodConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kLeftHoodConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kLeftHoodConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 20.0;
  }

  public static final ServoMotorSubsystemConfig kLeftTurretConfig = new ServoMotorSubsystemConfig();

  static {
    kLeftTurretConfig.name = "Left Turret";
    kLeftTurretConfig.talonCANID = new CANDeviceId(22, TunerConstants.kCANBus1.getName());

    kLeftTurretConfig.unitToRotorRatio = ScorerConstants.kTurretUnitToRotorRatio;
    kLeftTurretConfig.kMaxPositionUnits = ScorerConstants.kTurretMaxPositionUnits;
    kLeftTurretConfig.kMinPositionUnits = ScorerConstants.kTurretMinPositionUnits;
    kLeftTurretConfig.momentOfInertia = ScorerConstants.kTurretMomentOfInertia;

    kLeftTurretConfig.fxConfig.Slot0.kP = 2.0;
    kLeftTurretConfig.fxConfig.Slot0.kD = 0.0;
    kLeftTurretConfig.fxConfig.Slot0.kV = 0.2;
    kLeftTurretConfig.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 80.0;
    kLeftTurretConfig.fxConfig.MotionMagic.MotionMagicAcceleration = 300.0;

    kLeftTurretConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    kLeftTurretConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        kLeftTurretConfig.kMaxPositionUnits / kLeftTurretConfig.unitToRotorRatio;
    kLeftTurretConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    kLeftTurretConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        kLeftTurretConfig.kMinPositionUnits / kLeftTurretConfig.unitToRotorRatio;

    kLeftTurretConfig.fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kLeftTurretConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kLeftTurretConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kLeftTurretConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 5.0;
  }

  public static final ServoMotorSubsystemConfig kLeftFlywheelConfig =
      new ServoMotorSubsystemConfig();

  static {
    kLeftFlywheelConfig.name = "Left Flywheel";
    kLeftFlywheelConfig.talonCANID = new CANDeviceId(24, TunerConstants.kCANBus1.getName());
    kLeftFlywheelConfig.momentOfInertia = 0.00132536;
    kLeftFlywheelConfig.unitToRotorRatio = (24.0 / 18.0) * 60; // gear ratio * 60 for RPM to RPS

    kLeftFlywheelConfig.fxConfig.Slot0.kP = 0.5;
    kLeftFlywheelConfig.fxConfig.Slot0.kS = 0.02;
    kLeftFlywheelConfig.fxConfig.Slot0.kV = 0.1;

    kLeftFlywheelConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kLeftFlywheelConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kLeftFlywheelConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kLeftFlywheelConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 80.0;
  }

  public static final ServoMotorSubsystemConfig kRightHoodConfig = new ServoMotorSubsystemConfig();

  static {
    kRightHoodConfig.name = "Right Hood";
    kRightHoodConfig.talonCANID = new CANDeviceId(28, TunerConstants.kCANBus1.getName());
    kRightHoodConfig.unitToRotorRatio = ScorerConstants.kHoodUnitToRotorRatio;
    kRightHoodConfig.kMaxPositionUnits = ScorerConstants.kHoodMaxPositionUnits;
    kRightHoodConfig.kMinPositionUnits = ScorerConstants.kHoodMinPositionUnits;
    kRightHoodConfig.momentOfInertia = ScorerConstants.kHoodMomentOfInertia;
    kRightHoodConfig.fxConfig.Slot0.kP = 2.0;
    kRightHoodConfig.fxConfig.Slot0.kD = 0.0;
    kRightHoodConfig.fxConfig.Slot0.kV = 0.2;
    kRightHoodConfig.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 80.0;
    kRightHoodConfig.fxConfig.MotionMagic.MotionMagicAcceleration = 300.0;

    kRightHoodConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    kRightHoodConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        kRightHoodConfig.kMaxPositionUnits / kRightHoodConfig.unitToRotorRatio;
    kRightHoodConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    kRightHoodConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        kRightHoodConfig.kMinPositionUnits / kRightHoodConfig.unitToRotorRatio;

    kRightHoodConfig.fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kRightHoodConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kRightHoodConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kRightHoodConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 20.0;
  }

  public static final ServoMotorSubsystemConfig kRightTurretConfig =
      new ServoMotorSubsystemConfig();

  static {
    kRightTurretConfig.name = "Right Turret";
    kRightTurretConfig.talonCANID = new CANDeviceId(27, TunerConstants.kCANBus1.getName());
    kRightTurretConfig.unitToRotorRatio = ScorerConstants.kTurretUnitToRotorRatio;
    kRightTurretConfig.kMaxPositionUnits = ScorerConstants.kTurretMaxPositionUnits;
    kRightTurretConfig.kMinPositionUnits = ScorerConstants.kTurretMinPositionUnits;
    kRightTurretConfig.momentOfInertia = ScorerConstants.kTurretMomentOfInertia;
    kRightTurretConfig.fxConfig.Slot0.kP = 2.0;
    kRightTurretConfig.fxConfig.Slot0.kD = 0.0;
    kRightTurretConfig.fxConfig.Slot0.kV = 0.2;
    kRightTurretConfig.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 80.0;
    kRightTurretConfig.fxConfig.MotionMagic.MotionMagicAcceleration = 300.0;

    kRightTurretConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    kRightTurretConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        kRightTurretConfig.kMaxPositionUnits / kRightTurretConfig.unitToRotorRatio;
    kRightTurretConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    kRightTurretConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        kRightTurretConfig.kMinPositionUnits / kRightTurretConfig.unitToRotorRatio;

    kRightTurretConfig.fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kRightTurretConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kRightTurretConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kRightTurretConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 5.0;
  }

  public static final ServoMotorSubsystemConfig kRightFlywheelConfig =
      new ServoMotorSubsystemConfig();

  static {
    kRightFlywheelConfig.name = "Right Flywheel";
    kRightFlywheelConfig.talonCANID = new CANDeviceId(29, TunerConstants.kCANBus1.getName());
    kRightFlywheelConfig.momentOfInertia = 0.00132536;
    kRightFlywheelConfig.unitToRotorRatio = (24.0 / 18.0) * 60; // gear ratio * 60 for RPM to RPS

    kRightFlywheelConfig.fxConfig.Slot0.kP = 0.5;
    kRightFlywheelConfig.fxConfig.Slot0.kS = 0.02;
    kRightFlywheelConfig.fxConfig.Slot0.kV = 0.1;

    kRightFlywheelConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kRightFlywheelConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kRightFlywheelConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kRightFlywheelConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 80.0;
  }

  // #endregion

  // #region Intake Subsystems
  public static final class IntakeConstants {

    public static final double kIntakePivotStowedPosition = 0.0; // degrees

    public static final double kIntakeDutyCycleIntake = 1.0;
    public static final double kIntakeDutyCycleExhaust = -0.75;

    // max motor speed (7500rpm) we are setting to 7000rpm then convert to system (divide by 3.55)
    // roughly 1900
    public static final double kIntakeVelocityRPM = 1900.0;
    public static final double kDeployVelocityRPM = -100.0;

    public static final double kIntakePivotToleranceRadians = 0.05;
    public static final double kIntakeRollerRadius = 0.0269875; // in m

    public static final double kIntakePivotCancoderOffset = kIsPracticeBot ? 0.411133 : -0.086914;

    public static final double kIntakeStowDegrees = 0.0;
    public static final double kIntakeDeployDegrees = 145.0;
  }

  public static final ServoMotorSubsystemConfig kIntakeRollerConfig =
      new ServoMotorSubsystemConfig();

  static {
    kIntakeRollerConfig.name = "Intake_Roller";
    kIntakeRollerConfig.talonCANID =
        new CANDeviceId(13, TunerConstants.kCANBus1.getName()); // Motor 1 (master)
    kIntakeRollerConfig.momentOfInertia = 0.00132536;
    kIntakeRollerConfig.unitToRotorRatio =
        (18.0 / 20.0) * (10.0 / 32.0) * 60.0; // gear ratio in RPM to RPS

    kIntakeRollerConfig.fxConfig.Slot0.kP = 0.5; // Increased from 0.5
    kIntakeRollerConfig.fxConfig.Slot0.kI = 0.0;
    kIntakeRollerConfig.fxConfig.Slot0.kD = 0.0;
    kIntakeRollerConfig.fxConfig.Slot0.kS = 0.02; // Increased from 0.02 - overcome friction
    kIntakeRollerConfig.fxConfig.Slot0.kV = 0.1; // Increased from 0.1 - velocity feedforward
    kIntakeRollerConfig.fxConfig.Slot0.kA = 0.0;

    kIntakeRollerConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kIntakeRollerConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kIntakeRollerConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 80.0;

    kIntakeRollerConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
  }

  public static final ServoMotorSubsystemWithCanCoderConfig kIntakePivotConfig =
      new ServoMotorSubsystemWithCanCoderConfig();

  static {
    kIntakePivotConfig.name = "Intake_Pivot";
    kIntakePivotConfig.talonCANID = new CANDeviceId(12, TunerConstants.kCANBus1.getName());
    kIntakePivotConfig.momentOfInertia = 0.01;

    // PID Slot 0 gains for MotionMagicVoltage
    // Design sheet: X44, gear ratio 5.454545:1, 40A stator, Motion Magic Position Control
    kIntakePivotConfig.fxConfig.Slot0.kP = 2.0; // Start conservative — tune up from here
    kIntakePivotConfig.fxConfig.Slot0.kI = 0.0;
    kIntakePivotConfig.fxConfig.Slot0.kD = 0.0;
    // kIntakePivotConfig.fxConfig.Slot0.kS = 0.25; // Static friction compensation (volts)
    kIntakePivotConfig.fxConfig.Slot0.kV = 0.12; // Velocity feedforward (volts per rot/s)
    kIntakePivotConfig.fxConfig.Slot0.kA = 0.0;

    // Motion Magic profile — units are rotor rotations/s and rotations/s²
    kIntakePivotConfig.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 10.0; // rot/s
    kIntakePivotConfig.fxConfig.MotionMagic.MotionMagicAcceleration = 300.0; // rot/s²

    // Units = degrees
    // TODO 1.8125 is a fudge factor. Something is different between CAD and Bravo robot
    kIntakePivotConfig.unitToRotorRatio =
        (12.0 / 32.0) * (16.0 / 38.0) * (16.0 / 40.0) * (12.0 / 18.0) * 360.0 * 1.8125;

    // Position limits in degrees — design sheet: 0 → 145 degrees
    kIntakePivotConfig.kMaxPositionUnits = 145.0; // degrees (fully deployed)
    kIntakePivotConfig.kMinPositionUnits = 0.0; // degrees (fully stowed)
    kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        kIntakePivotConfig.kMaxPositionUnits / kIntakePivotConfig.unitToRotorRatio;
    kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        kIntakePivotConfig.kMinPositionUnits / kIntakePivotConfig.unitToRotorRatio;

    kIntakePivotConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kIntakePivotConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kIntakePivotConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kIntakePivotConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 40.0; // Per design sheet
  }

  // #endregion

  // #region Agitator Subsystems

  public static final class AgitatorConstants {
    // Floor Roller speeds (Velocity Voltage Control)
    // Output Top Speed = 75 RPS (4500 RPM) from design sheet
    // TESTING: Increased speed to make velocity control more noticeable
    public static final double kFloorRollerSnowblowRPM =
        2000.0; // RPM at output (increased for testing)
    public static final double kFloorRollerCollectRPM =
        200.0; // RPM while intaking (Decreased for Collect mode)
    public static final double kFloorRollerSnowblowRPS =
        kFloorRollerSnowblowRPM / 60.0; // RPS at output = 30 RPS
    public static final double kFloorRollerCollectRPS =
        kFloorRollerCollectRPM / 60.0; // RPS at output = 6.67 RPS
    // Vertical Feed Roller speeds (Velocity Voltage Control)
    // Output Top Speed = 83.33 RPS (5000 RPM) from design sheet
    public static final double kVerticalFeedIntakeRPM = 3600.0; // RPM at output
    public static final double kVerticalFeedOuttakeRPM = -3600.0; // RPM at output
    public static final double kVerticalFeedIntakeRPS =
        kVerticalFeedIntakeRPM / 60.0; // RPS at output
    public static final double kVerticalFeedOuttakeRPS =
        kVerticalFeedOuttakeRPM / 60.0; // RPS at output
  }

  // ---- Right Floor Roller (CAN 25, CANivore #2) ----
  // Design Sheet: X44, gear ratio 1.66667:1 (20/12), 75 RPS output, Outtake direction, 80A
  public static final ServoMotorSubsystemConfig kRightFloorRollerConfig =
      new ServoMotorSubsystemConfig();

  static {
    kRightFloorRollerConfig.name = "RightFloorRoller";
    kRightFloorRollerConfig.talonCANID = new CANDeviceId(25, TunerConstants.kCANBus1.getName());
    kRightFloorRollerConfig.momentOfInertia = 0.00132536;
    kRightFloorRollerConfig.unitToRotorRatio = (12.0 / 120.0) * 60; // gear ratio 1.66667:1

    // Velocity PID gains - SIGNIFICANTLY INCREASED for better response
    // Phoenix Tuner confirmed velocity control works at these higher gains
    kRightFloorRollerConfig.fxConfig.Slot0.kP = 0.5; // Increased from 0.5
    kRightFloorRollerConfig.fxConfig.Slot0.kI = 0.0;
    kRightFloorRollerConfig.fxConfig.Slot0.kD = 0.0;
    kRightFloorRollerConfig.fxConfig.Slot0.kS = 0.02; // Increased from 0.02 - overcome friction
    kRightFloorRollerConfig.fxConfig.Slot0.kV = 0.1; // Increased from 0.1 - velocity feedforward
    kRightFloorRollerConfig.fxConfig.Slot0.kA = 0.0;

    kRightFloorRollerConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    kRightFloorRollerConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kRightFloorRollerConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kRightFloorRollerConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 80.0;
  }

  // ---- Left Floor Roller (CAN 20, CANivore #2) ----
  // Design Sheet: X44, gear ratio 1.66667:1 (20/12), 75 RPS output, Intake direction, 80A
  public static final ServoMotorSubsystemConfig kLeftFloorRollerConfig =
      new ServoMotorSubsystemConfig();

  static {
    kLeftFloorRollerConfig.name = "LeftFloorRoller";
    kLeftFloorRollerConfig.talonCANID = new CANDeviceId(20, TunerConstants.kCANBus1.getName());
    kLeftFloorRollerConfig.momentOfInertia = 0.00132536;
    kLeftFloorRollerConfig.unitToRotorRatio = (12.0 / 120.0) * 60; // gear ratio 1.66667:1

    kLeftFloorRollerConfig.fxConfig.Slot0.kP = 0.5;
    kLeftFloorRollerConfig.fxConfig.Slot0.kS = 0.02;
    kLeftFloorRollerConfig.fxConfig.Slot0.kV = 0.1;

    kLeftFloorRollerConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kLeftFloorRollerConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kLeftFloorRollerConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 80.0;
  }

  // ---- Right Vertical Feed Roller (CAN 26, CANivore #2) ----
  // Design Sheet: X44, gear ratio 1.5:1 (18/12), 83.33 RPS output, Outtake direction, 80A
  public static final ServoMotorSubsystemConfig kRightVerticalFeedConfig =
      new ServoMotorSubsystemConfig();

  static {
    kRightVerticalFeedConfig.name = "RightVerticalFeed";
    kRightVerticalFeedConfig.talonCANID = new CANDeviceId(26, TunerConstants.kCANBus1.getName());
    kRightVerticalFeedConfig.momentOfInertia = 0.00132536;
    kRightVerticalFeedConfig.unitToRotorRatio = (12.0 / 18.0) * 60; // gear ratio 1.5:1

    kRightVerticalFeedConfig.fxConfig.Slot0.kP = 0.5;
    kRightVerticalFeedConfig.fxConfig.Slot0.kS = 0.02;
    kRightVerticalFeedConfig.fxConfig.Slot0.kV = 0.1;

    kRightVerticalFeedConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kRightVerticalFeedConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kRightVerticalFeedConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kRightVerticalFeedConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 80.0;
  }

  // ---- Left Vertical Feed Roller (CAN 21, CANivore #2) ----
  // Design Sheet: X44, gear ratio 1.5:1 (18/12), 83.33 RPS output, Intake direction, 80A
  public static final ServoMotorSubsystemConfig kLeftVerticalFeedConfig =
      new ServoMotorSubsystemConfig();

  static {
    kLeftVerticalFeedConfig.name = "LeftVerticalFeed";
    kLeftVerticalFeedConfig.talonCANID = new CANDeviceId(21, TunerConstants.kCANBus1.getName());
    kLeftVerticalFeedConfig.momentOfInertia = 0.00132536;
    kLeftVerticalFeedConfig.unitToRotorRatio = (12.0 / 18.0) * 60; // gear ratio 1.5:1

    kLeftVerticalFeedConfig.fxConfig.Slot0.kP = 0.5;
    kLeftVerticalFeedConfig.fxConfig.Slot0.kS = 0.02;
    kLeftVerticalFeedConfig.fxConfig.Slot0.kV = 0.1;

    kLeftVerticalFeedConfig.fxConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    kLeftVerticalFeedConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    kLeftVerticalFeedConfig.fxConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    kLeftVerticalFeedConfig.fxConfig.CurrentLimits.StatorCurrentLimit = 80.0;
  }

  // #endregion

  /**
   * Check if this system has a certain mac address in any network device.
   *
   * @param mac_address Mac address to check.
   * @return true if some device with this mac address exists on this system.
   */
  public static boolean hasMacAddress(final String mac_address) {
    try {
      Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
      while (nwInterface.hasMoreElements()) {
        NetworkInterface nis = nwInterface.nextElement();
        if (nis == null) {
          continue;
        }
        StringBuilder device_mac_sb = new StringBuilder();
        System.out.println("hasMacAddress: NIS: " + nis.getDisplayName());
        byte[] mac = nis.getHardwareAddress();
        if (mac != null) {
          for (int i = 0; i < mac.length; i++) {
            device_mac_sb.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? ":" : ""));
          }
          String device_mac = device_mac_sb.toString();
          System.out.println(
              "hasMacAddress: NIS " + nis.getDisplayName() + " device_mac: " + device_mac);
          if (mac_address.equals(device_mac)) {
            System.out.println("hasMacAddress: ** Mac address match! " + device_mac);
            return true;
          }
        } else {
          System.out.println("hasMacAddress: Address doesn't exist or is not accessible");
        }
      }

    } catch (SocketException e) {
      e.printStackTrace();
    }
    return false;
  }
}
