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
import edu.wpi.first.math.util.Units;
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

  public static final String kPracticeBotMacAddress = "00:80:2F:33:BF:BB";
  public static boolean kIsPracticeBot = hasMacAddress(kPracticeBotMacAddress);

  // Set this to select which robot's tuner constants to use
  public static final Bot currentBot = Bot.SOFTWARE;

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
    SOFTWARE
    // Add more robot variants here as needed, e.g.:
    // COMPETITION,
    // PRACTICE
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

  public static final class IntakeConstants {
    public static final double kIntakeDutyCycleIntake = 1.0;
    public static final double kIntakeDutyCycleExhaust = -0.75;

    public static final double kIntakeVelocityInRPM = 1000.0;
    public static final double kIntakeVelocityOutRPM = 500.0;

    public static final double kIntakePivotStowPositionRadians = Units.degreesToRadians(-80);
    public static final double kIntakePivotStowForClimbPositionRadians =
        Units.degreesToRadians(-90);
    public static final double kIntakePivotDeployPositionRadians = 0.0;
    public static final double kIntakePivotCoralPushedPositionRadians =
        Units.degreesToRadians(-0.5);
    public static final double kIntakePivotLollipopDeployPositionRadians =
        Units.degreesToRadians(30);
    public static final double kIntakePivotToleranceRadians = 0.05;
    public static final double kIntakeRollerRadius = 0.0269875; // in m

    public static final double kIntakePivotCancoderOffset = kIsPracticeBot ? 0.411133 : -0.086914;
  }

  public static final ServoMotorSubsystemConfig kIntakeRollerConfig =
      new ServoMotorSubsystemConfig();

  static {
    kIntakeRollerConfig.name = "Intake_Roller";
    kIntakeRollerConfig.talonCANID = new CANDeviceId(58, TunerConstants.kCANBusRio.getName());
    kIntakeRollerConfig.momentOfInertia = 0.00132536;
    kIntakeRollerConfig.unitToRotorRatio = 60.0;

    kIntakeRollerConfig.fxConfig.Slot0.kP = 0.5;
    kIntakeRollerConfig.fxConfig.Slot0.kS = 0.02;
    kIntakeRollerConfig.fxConfig.Slot0.kV = 0.1;

    kIntakeRollerConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kIntakeRollerConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kIntakeRollerConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
  }

  public static final ServoMotorSubsystemWithCanCoderConfig kIntakePivotConfig =
      new ServoMotorSubsystemWithCanCoderConfig();

  static {
    kIntakePivotConfig.name = "Intake_Pivot";
    kIntakePivotConfig.talonCANID = new CANDeviceId(23, TunerConstants.kCANBus.getName());
    kIntakePivotConfig.momentOfInertia = 0.01;
    kIntakePivotConfig.fxConfig.Slot0.kP = 2.0;
    kIntakePivotConfig.fxConfig.Slot0.kD = 0.3;
    kIntakePivotConfig.fxConfig.Slot0.kV = 0.15;
    kIntakePivotConfig.fxConfig.MotionMagic.MotionMagicCruiseVelocity = 80.0;
    kIntakePivotConfig.fxConfig.MotionMagic.MotionMagicAcceleration = 300.0;
    kIntakePivotConfig.unitToRotorRatio =
        Units.rotationsToRadians((10.0 / 36.0) * (14.0 / 42.0) * (14.0 / 56.0));

    kIntakePivotConfig.canCoderConfig.CANID = new CANDeviceId(30, TunerConstants.kCANBus.getName());
    kIntakePivotConfig.canCoderConfig.config.MagnetSensor.MagnetOffset =
        IntakeConstants.kIntakePivotCancoderOffset;
    kIntakePivotConfig.cancoderToUnitsRatio = Units.rotationsToRadians(1);

    kIntakePivotConfig.kMaxPositionUnits = 0.0;
    kIntakePivotConfig.kMinPositionUnits = Units.degreesToRadians(-115.6);
    kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        kIntakePivotConfig.kMaxPositionUnits / kIntakePivotConfig.unitToRotorRatio;
    kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    kIntakePivotConfig.fxConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        kIntakePivotConfig.kMinPositionUnits / kIntakePivotConfig.unitToRotorRatio;

    kIntakePivotConfig.fxConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    kIntakePivotConfig.fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    kIntakePivotConfig.fxConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    kIntakePivotConfig.fxConfig.CurrentLimits.SupplyCurrentLimit = 60.0;
  }

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
