package frc.robot.Actors.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDstrip {
    
    private final AddressableLED m_led;
    private AddressableLEDBuffer ledBuffer;
    private LEDPattern curPattern;
    private double animIndex = 0;

    public LEDstrip(int port, int length) {
        m_led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);
        m_led.setData(ledBuffer);
        m_led.start();
    }

    public void setPattern(LEDPattern pattern) {
        curPattern = pattern;
        animIndex = 0;
    }

    public void periodic() {
        animIndex = (animIndex%curPattern.loopSeconds())+0.02;
        ledBuffer = curPattern.getLEDs(0, ledBuffer);
        m_led.setData(ledBuffer);
    }
}
