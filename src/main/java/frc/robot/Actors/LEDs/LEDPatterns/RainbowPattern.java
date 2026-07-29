package frc.robot.Actors.LEDs.LEDPatterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Actors.LEDs.LEDPattern;
import frc.robot.Utils.LEDHelpers;

public class RainbowPattern implements LEDPattern {
    private final int length;

    public RainbowPattern(int length) {
        this.length = length;
    }

    public AddressableLEDBuffer getLEDs(double index, AddressableLEDBuffer oldBuff) {

        for (int i = 0; i < length; i++) {
                    LEDHelpers.setLED(oldBuff, i,
                            LEDHelpers.hsvToRgb(new double[] {
                                    (((index) + (i / length)) * 360) % 360.0, 1.0, 1.0 }));
                }
        return null;
    }

    public double loopSeconds() {
        return 1.0;
    }
}