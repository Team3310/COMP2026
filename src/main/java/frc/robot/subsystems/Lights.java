package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import java.util.Map;

public class Lights extends SubsystemBase {
  private static Lights instance;
  private LightMode mode = LightMode.GYRO_RESETING; // Default mode
  private AddressableLED ledStrip;
  private AddressableLEDBuffer ledBuffer;

  public static Lights getInstance() {
    if (instance == null) {
      instance = new Lights();
    }
    return instance;
  }

  private Lights() {

    ledStrip = new AddressableLED(1);
    ledBuffer = new AddressableLEDBuffer(43); // Number of LEDs in the strip

    ledStrip.setLength(ledBuffer.getLength());
    LEDPattern.solid(Color.kRed).applyTo(ledBuffer);
    ledStrip.setColorOrder(AddressableLED.ColorOrder.kGRB);
    ledStrip.setData(ledBuffer);
    ledStrip.start();
    register();
  }

  Timer gyroLightTimer = new Timer();

  public void setMode(LightMode mode) {
    if (mode == LightMode.GYRO_STOP
        || mode == LightMode.GYRO_RESETING
        || mode == LightMode.GYRO_NONRESETING) {
      gyroLightTimer.restart();
      this.mode = mode;
    } else if ((this.mode == LightMode.GYRO_STOP
            || this.mode == LightMode.GYRO_RESETING
            || this.mode == LightMode.GYRO_NONRESETING)
        && gyroLightTimer.hasElapsed(2.0)) {
      this.mode = mode;
    } else if (!(this.mode == LightMode.GYRO_STOP
        || this.mode == LightMode.GYRO_RESETING
        || this.mode == LightMode.GYRO_NONRESETING)) {
      this.mode = mode;
    }
  }

  @Override
  public void periodic() {
    switch (mode) {
      case AUTO_POSE_BAD:
        int r, g, b = 0;
        double xError = SmartDashboard.getNumber("pose x error", 0.0);
        double yError = SmartDashboard.getNumber("pose y error", 0.0);
        double rotationError = SmartDashboard.getNumber("rotation error", 0.0);

        if (Math.abs(xError) < 0.1) {
          r = 0;
          g = 255;
          b = 255;
        } else if (xError < 0.0) { // X error negative means go towards blue alliance wall
          r = 0;
          g = 0;
          b = 255;
        } else { // X error positive means go towards red alliance wall
          r = 255;
          g = 0;
          b = 0;
        }

        int ry, gy, by = 0;

        if (Math.abs(yError) < 0.1) {
          ry = 0;
          gy = 255;
          by = 255;
        } else if (yError > 0.0) { // Y error positive means go towards blue barge
          // yellow ish
          ry = 0;
          gy = 0;
          by = 255;
        } else { // Y error negative means go towards red barge
          ry = 255;
          gy = 0;
          by = 0;
        }

        int rR, gR, bR = 0;

        if (Math.abs(rotationError) < 1.0) {
          rR = 0;
          gR = 255;
          bR = 255;
        } else if (rotationError > 0.0) { // blue means CCW
          rR = 0;
          gR = 0;
          bR = 255;
        } else if (rotationError < 0.0) { // red means CW
          rR = 255;
          gR = 0;
          bR = 0;
        } else {
          rR = 0;
          gR = 0;
          bR = 0;
        }

        LEDPattern.steps(
                Map.of(
                    0,
                    new Color(r, g, b), // bottom - x - alliance walls
                    0.45,
                    new Color(ry, gy, by),
                    0.9,
                    new Color(rR, gR, bR) // top - y - barges
                    ))
            .applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        break;
      case AUTO_POSE_GOOD:
        LEDPattern.solid(new Color(0, 255, 255))
            .atBrightness(Percent.of(Robot.inPit ? 0.85 : 1.0))
            .applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        break;
      case GYRO_STOP:
        (Robot.inPit
                ? LEDPattern.solid(new Color(0, 0, 255)).atBrightness(Percent.of(1.0))
                : LEDPattern.solid(new Color(0, 0, 255)))
            .applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        break;
      case GYRO_NONRESETING:
        LEDPattern pattern = LEDPattern.solid(new Color(0, 255, 0));
        (Robot.inPit ? pattern.atBrightness(Percent.of(0.85)) : pattern).applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        break;
      case GYRO_RESETING:
        (Robot.inPit
                ? LEDPattern.solid(new Color(255, 0, 0))
                    .atBrightness(Percent.of(Robot.inPit ? 0.85 : 1.0))
                : LEDPattern.solid(new Color(255, 0, 0)))
            .applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        break;
      case AUTO_POSE_STOP:
        (Robot.inPit
                ? LEDPattern.solid(new Color(0, 0, 255))
                    .atBrightness(Percent.of(Robot.inPit ? 0.85 : 1.0))
                : LEDPattern.solid(new Color(0, 0, 255)))
            .applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        break;
      case OFF:
        LEDPattern.kOff.applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        break;
      case BOT_STATE:
        // Check if in pit mode - if so, make entire strip blue
        if (Robot.inPit) {
          LEDPattern.solid(new Color(0, 0, 255)).applyTo(ledBuffer);
          ledStrip.setData(ledBuffer);
        } else {
          // Bottom half (0-21 LEDs) changes color based on bot state
          LEDPattern.solid(new Color(0, 0, 0)).applyTo(ledBuffer);
          Color bottomColor;

          switch (Robot.currentState) {
            case SNOWBLOW:
              // White for snowblow mode
              bottomColor = new Color(255, 255, 255);
              break;
            case COLLECT:
              // Blue for collect mode
              bottomColor = new Color(0, 0, 255);
              break;
            case DEFENCEIN:
              // Red for defense mode
              bottomColor = new Color(255, 0, 0);
              break;
            case DEFENCEOUT:
              // Yellow for defense mode
              bottomColor = new Color(255, 255, 0);
              break;
            default:
              // Off for other states
              bottomColor = new Color(0, 0, 0);
              break;
          }

          // Apply color to bottom half (LEDs 0-21)
          LEDPattern.solid(bottomColor).applyTo(ledBuffer.createView(0, 21));
          ledStrip.setData(ledBuffer);
        }
        break;
      default:
        LEDPattern.kOff.applyTo(ledBuffer);
        ledStrip.setData(ledBuffer);
        break;
    }
  }

  public enum LightMode {
    GYRO_RESETING,
    GYRO_NONRESETING,
    GYRO_STOP,
    AUTO_POSE_BAD,
    AUTO_POSE_GOOD,
    AUTO_POSE_STOP,
    BOT_STATE,
    OFF;
  }
}
