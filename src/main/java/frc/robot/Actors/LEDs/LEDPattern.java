package frc.robot.Actors.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface LEDPattern {
    public AddressableLEDBuffer getLEDs(double index, AddressableLEDBuffer oldBuff);
    public double loopSeconds();
}
