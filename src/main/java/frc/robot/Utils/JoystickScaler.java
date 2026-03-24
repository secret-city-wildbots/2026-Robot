package frc.robot.Utils;

public class JoystickScaler {
    /**
     * Scale joystick input for driving
     * @param input [0,1]
     * @return [0,1]
     */
    public static double scaleStrafe(double input) {
        return Math.signum(input)*(Math.abs(input) > 0.005 ? Math.pow(Math.abs(input),3.0):0.0);
    }

    public static double scaleRotate(double input) {
        return Math.signum(input)*(Math.abs(input) > 0.01 ? Math.pow(Math.abs(input),2.3):0.0);
    }
}