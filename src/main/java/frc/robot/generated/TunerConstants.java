package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

/**
 * Facade that delegates to the appropriate bot-specific TunerConstants based on
 * Constants.currentBot. This allows you to switch between different robot configurations by
 * changing one variable.
 */
public class TunerConstants {
  // Delegated constants - these forward to the selected variant based on Constants.currentBot
  public static final CANBus kCANBus;
  public static final CANBus kCANBusRio;
  public static final LinearVelocity kSpeedAt12Volts;
  public static final SwerveDrivetrainConstants DrivetrainConstants;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontLeft;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      FrontRight;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackLeft;
  public static final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      BackRight;

  // Static initializer block - runs once when class is loaded, switches based on currentBot
  static {
    switch (Constants.currentBot) {
      case SOFTWARE:
        kCANBus = TunerConstantsSoftware.kCANBus;
        kCANBusRio = TunerConstantsSoftware.kCANBusRio;
        kSpeedAt12Volts = TunerConstantsSoftware.kSpeedAt12Volts;
        DrivetrainConstants = TunerConstantsSoftware.DrivetrainConstants;
        FrontLeft = TunerConstantsSoftware.FrontLeft;
        FrontRight = TunerConstantsSoftware.FrontRight;
        BackLeft = TunerConstantsSoftware.BackLeft;
        BackRight = TunerConstantsSoftware.BackRight;
        break;
        // Add more cases here for additional robots:
        // case COMPETITION:
        //   kCANBus = TunerConstantsCompetition.kCANBus;
        //   kSpeedAt12Volts = TunerConstantsCompetition.kSpeedAt12Volts;
        //   DrivetrainConstants = TunerConstantsCompetition.DrivetrainConstants;
        //   FrontLeft = TunerConstantsCompetition.FrontLeft;
        //   FrontRight = TunerConstantsCompetition.FrontRight;
        //   BackLeft = TunerConstantsCompetition.BackLeft;
        //   BackRight = TunerConstantsCompetition.BackRight;
        //   break;
      default:
        throw new IllegalStateException("Unknown bot: " + Constants.currentBot);
    }
  }

  /** Swerve Drive class utilizing CTR Electronics' Phoenix 6 API with the selected device types. */
  public static class TunerSwerveDrivetrain extends TunerConstantsSoftware.TunerSwerveDrivetrain {
    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
      super(drivetrainConstants, modules);
    }

    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(drivetrainConstants, odometryUpdateFrequency, modules);
    }

    public TunerSwerveDrivetrain(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1>
            odometryStandardDeviation,
        edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1>
            visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules) {
      super(
          drivetrainConstants,
          odometryUpdateFrequency,
          odometryStandardDeviation,
          visionStandardDeviation,
          modules);
    }
  }
}
