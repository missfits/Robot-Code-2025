package frc.robot.subsystems;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  private static final int kPort = LEDConstants.KPORT;
  private static final int kLength = LEDConstants.KLENGTH;

  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;

  /** Called once at the beginning of the robot program. */
  public LEDSubsystem() {
    // PWM port 9
    // Must be a PWM header, not MXP or DIO
    m_led = new AddressableLED(LEDConstants.KPORT);

    // Reuse buffer
    // Default to a length of 60, start empty output
    // Length is expensive to set, so only set it once, then just update data
    m_ledBuffer = new AddressableLEDBuffer(LEDConstants.KLENGTH);
    m_led.setLength(m_ledBuffer.getLength());

    // Set the data
    m_led.setData(m_ledBuffer);

    m_led.start();

    // Set the default command to turn the strip off, otherwise the last colors written by
    // the last command to run will continue to be displayed.
    // Note: Other default patterns could be used instead!
    setDefaultCommand((runGradientBlueYellow()).withName("Off"));
  }


  @Override
  public void periodic() {
    // Periodically send the latest LED color data to the LED strip for it to display


    m_led.setData(m_ledBuffer);
  }

  /**
   * Creates a command that runs a pattern on the entire LED strip.
   *
   * @param pattern the LED pattern to run
   */
  public Command runPattern(LEDPattern pattern) {
    return run(() -> pattern.applyTo(m_ledBuffer)).ignoringDisable(true);
  }

  public Command runBlinkGreen() {
    return runPattern(LEDPattern.solid(Color.kGreen).blink(Time.ofBaseUnits(0.25, Units.Seconds)));
  }

  public Command runSolidYellow() {
    return runPattern(LEDPattern.solid(Color.kYellow));
  }

  public Command runSolidBlue() {
    return runPattern(LEDPattern.solid(Color.kDarkBlue));
  }

  public Command runSolidOrange() {
    return runPattern(LEDPattern.solid(Color.kOrange));
  }

  public Command runSolidGreen() {
    return runPattern(LEDPattern.solid(Color.kGreen));
  }

  public Command runSolidRed() {
    return runPattern(LEDPattern.solid(Color.kRed));
  }

  public Command runSolidWhite() {
    return runPattern(LEDPattern.solid(Color.kWhite));
  }

  public Command runSolidPink() {
    return runPattern(LEDPattern.solid(Color.kPink));
  }

  public Command runSolidPurple() {
    return runPattern(LEDPattern.solid(Color.kPurple));
  }
  
  public Command runGradientBlueYellow(){
    LEDPattern gradient = LEDPattern.gradient(LEDPattern.GradientType.kDiscontinuous, Color.kYellow, Color.kBlue);
    return runPattern(gradient);
  }
  
}