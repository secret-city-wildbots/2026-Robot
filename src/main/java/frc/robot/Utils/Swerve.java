package frc.robot.Utils;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Swerve {
    /**
     * Creates an array of configurations for TalonFXs with the following changes
     * from default:
     * <ul>
     * <li>Rotation directions are specified based on module position
     * <li>Forward and reverse limits are disabled due to the shifter sensors being
     * plugged into these ports in some versions
     * <li>Neutral mode is set to brake so that the robot stops when prompted
     * <li>Open and closed loop ramps are both 0.1 to get consistent acceleration
     * decceleration without drawing too much current
     * </ul>
     * 
     * Where element 0 matches module 0, element 1 matches module 1, etc...
     * 
     * @return an array of drive configurations
    */
    public static TalonFXConfiguration[] swerveModuleDriveConfigs() {
    TalonFXConfiguration[] configs = new TalonFXConfiguration[4];

    // Front right
    configs[0] = new TalonFXConfiguration();
    configs[0].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[0].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[0].MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs[0].OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    configs[0].ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    configs[0].MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    // Front Left
    configs[1] = new TalonFXConfiguration();
    configs[1].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[1].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[1].MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs[1].OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    configs[1].ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    configs[1].MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Back left
    configs[2] = new TalonFXConfiguration();
    configs[2].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[2].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[2].MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs[2].OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    configs[2].ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    configs[2].MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Back right
    configs[3] = new TalonFXConfiguration();
    configs[3].HardwareLimitSwitch.ForwardLimitEnable = false;
    configs[3].HardwareLimitSwitch.ReverseLimitEnable = false;
    configs[3].MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs[3].OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
    configs[3].ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
    configs[3].MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    return configs;
  }

  /**
   * Creates an array of configurations for TalonFXs with the following changes
   * from default:
   * <ul>
   * <li>Rotation direction is set to Clockwise_Positive
   * <li>Neutral mode is set to brake so that the robot stops when prompted
   * <li>Open and closed loop ramps are both 0.1 to get consistent acceleration
   * decceleration without drawing too much current
   * </ul>
   * 
   * Where element 0 matches module 0, element 1 matches module 1, etc...
   * 
   * @return an array of azimuth configurations
   */
  public static TalonFXConfiguration[] swerveModuleAzimuthConfigs() {
    TalonFXConfiguration[] configs = new TalonFXConfiguration[4];
    InvertedValue inverted = InvertedValue.CounterClockwise_Positive;
    for (int i = 0; i < 4; i++) {
      configs[i] = new TalonFXConfiguration();
      configs[i].MotorOutput.NeutralMode = NeutralModeValue.Brake;
      configs[i].OpenLoopRamps.DutyCycleOpenLoopRampPeriod = 0.1;
      configs[i].ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.1;
      configs[i].MotorOutput.Inverted = inverted;
    }

    return configs;
  }
}
