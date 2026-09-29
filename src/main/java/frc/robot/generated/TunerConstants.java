package frc.robot.generated;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import edu.wpi.first.units.measure.*;
import frc.robot.Constants;

/**
 * Facade that delegates to the COMP or PRACTICE Tuner X constants based on {@link
 * Constants#currentBot}. To switch robots, change {@code currentBot} at the top of Constants.java.
 */
public class TunerConstants {
  public static final CANBus kCANBus1;
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

  static {
    switch (Constants.currentBot) {
      case COMP:
        kCANBus1 = TunerConstantsComp.kCANBus1;
        kSpeedAt12Volts = TunerConstantsComp.kSpeedAt12Volts;
        DrivetrainConstants = TunerConstantsComp.DrivetrainConstants;
        FrontLeft = TunerConstantsComp.FrontLeft;
        FrontRight = TunerConstantsComp.FrontRight;
        BackLeft = TunerConstantsComp.BackLeft;
        BackRight = TunerConstantsComp.BackRight;
        break;
      case PRACTICE:
        kCANBus1 = TunerConstantsPractice.kCANBus1;
        kSpeedAt12Volts = TunerConstantsPractice.kSpeedAt12Volts;
        DrivetrainConstants = TunerConstantsPractice.DrivetrainConstants;
        FrontLeft = TunerConstantsPractice.FrontLeft;
        FrontRight = TunerConstantsPractice.FrontRight;
        BackLeft = TunerConstantsPractice.BackLeft;
        BackRight = TunerConstantsPractice.BackRight;
        break;
      default:
        throw new IllegalStateException("Unknown bot: " + Constants.currentBot);
    }
  }
}
