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
  public static final CANBus kCANBus1;
  public static final CANBus kCANBus2;
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
        kCANBus1 = TunerConstantsSoftware.kCANBus1;
        kCANBus2 = TunerConstantsSoftware.kCANBus2;
        kCANBusRio = TunerConstantsSoftware.kCANBusRio;
        kSpeedAt12Volts = TunerConstantsSoftware.kSpeedAt12Volts;
        DrivetrainConstants = TunerConstantsSoftware.DrivetrainConstants;
        FrontLeft = TunerConstantsSoftware.FrontLeft;
        FrontRight = TunerConstantsSoftware.FrontRight;
        BackLeft = TunerConstantsSoftware.BackLeft;
        BackRight = TunerConstantsSoftware.BackRight;
        break;
      case BRAVO:
        kCANBus1 = TunerConstantsBravo.kCANBus1;
        kCANBusRio = TunerConstantsBravo.kCANBusRio;
        kSpeedAt12Volts = TunerConstantsBravo.kSpeedAt12Volts;
        DrivetrainConstants = TunerConstantsBravo.DrivetrainConstants;
        FrontLeft = TunerConstantsBravo.FrontLeft;
        FrontRight = TunerConstantsBravo.FrontRight;
        BackLeft = TunerConstantsBravo.BackLeft;
        BackRight = TunerConstantsBravo.BackRight;
        break;
      case COMP:
        kCANBus1 = TunerConstantsBravo.kCANBus1;
        kCANBus2 = TunerConstantsBravo.kCANBus2;
        kCANBusRio = TunerConstantsBravo.kCANBusRio;
        kSpeedAt12Volts = TunerConstantsBravo.kSpeedAt12Volts;
        DrivetrainConstants = TunerConstantsBravo.DrivetrainConstants;
        FrontLeft = TunerConstantsBravo.FrontLeft;
        FrontRight = TunerConstantsBravo.FrontRight;
        BackLeft = TunerConstantsBravo.BackLeft;
        BackRight = TunerConstantsBravo.BackRight;
        break;
      case PRACTICE:
        kCANBus1 = TunerConstantsPractice.kCANBus1;
        kCANBus2 = TunerConstantsPractice.kCANBus2;
        kCANBusRio = TunerConstantsPractice.kCANBusRio;
        kSpeedAt12Volts = TunerConstantsPractice.kSpeedAt12Volts;
        DrivetrainConstants = TunerConstantsPractice.DrivetrainConstants;
        FrontLeft = TunerConstantsPractice.FrontLeft;
        FrontRight = TunerConstantsPractice.FrontRight;
        BackLeft = TunerConstantsPractice.BackLeft;
        BackRight = TunerConstantsPractice.BackRight;
        break;
        // Add more cases here for additional robots:
        // case COMPETITION:
        //   kCANBus1 = TunerConstantsCompetition.kCANBus1;
        //   kCANBus2 = TunerConstantsCompetition.kCANBus2;
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
