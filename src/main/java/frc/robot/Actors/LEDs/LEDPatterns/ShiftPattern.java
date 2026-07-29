package frc.robot.Actors.LEDs.LEDPatterns;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Actors.LEDs.LEDPattern;
import frc.robot.Utils.LEDHelpers;

public class ShiftPattern implements LEDPattern {
    private final int length;

    public ShiftPattern(int length) {
        this.length = length;
    }

    public AddressableLEDBuffer getLEDs(double index, AddressableLEDBuffer oldBuff) {
        int offset = (int) Math.floor(index * 2);

        for (int i = 0; i < length; i++) {
            if (i <= length / 4) {
                LEDHelpers.setLED(oldBuff, i + offset, LEDHelpers.teamColors[0]);
            } else if (i <= length / 2 && i > length / 4) {
                LEDHelpers.setLED(oldBuff, i + offset, LEDHelpers.teamColors[1]);
            } else if (i <= length / 4 * 3 && i > length / 2) {
                LEDHelpers.setLED(oldBuff, i + offset, LEDHelpers.teamColors[2]);
            } else {
                LEDHelpers.setLED(oldBuff, i + offset, LEDHelpers.teamColors[3]);
            }
        }

        for (int i = 0; i < length; i++) {
            if (i <= length / 4) {
                LEDHelpers.setLED(oldBuff, i + offset - length, LEDHelpers.teamColors[0]);
            } else if (i <= length / 2 && i > length / 4) {
                LEDHelpers.setLED(oldBuff, i + offset - length, LEDHelpers.teamColors[1]);
            } else if (i <= length / 4 * 3 && i > length / 2) {
                LEDHelpers.setLED(oldBuff, i + offset - length, LEDHelpers.teamColors[2]);
            } else {
                LEDHelpers.setLED(oldBuff, i + offset - length, LEDHelpers.teamColors[3]);
            }
        }
        return null;
    }

    public double loopSeconds() {
        return 1.0;
    }
}