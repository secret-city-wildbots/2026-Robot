package frc.robot.Actors;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import frc.robot.Utils.MotorType;
import frc.robot.Utils.RotationDir;

public class Motor {
    public MotorType type;
    public TalonFX motorTFX;
    public TalonFXConfiguration configTFX;
    public Slot0Configs slot0TFX;
    public SparkMax motorSPX;
    public SparkMaxConfig configSPX;
    public MotorConfig motorConfig;
    public String actuatorName = "not_set";
    public int CanID;

    public Motor(int CanID, MotorType type) {
        this.CanID = CanID;
        this.type = type;

        switch (type) {
            case SPX:
                this.motorSPX = new SparkMax(CanID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
                this.configSPX = new SparkMaxConfig();
                break;
            case TFX:
                this.motorTFX = new TalonFX(CanID);
                this.configTFX = new TalonFXConfiguration();
                this.slot0TFX = new Slot0Configs();
                this.motorTFX.getConfigurator().setPosition(0);
                break;
            case None:
                System.err.println("Motor initialized with None type with CanID " + this.CanID);
        }

        this.motorConfig = new MotorConfig();
        this.applyConfig();
    }

    public Motor(int CanID, MotorType type, String actuatorName) {
        this.CanID = CanID;
        this.type = type;
        this.actuatorName = actuatorName;

        switch (type) {
            case SPX:
                this.motorSPX = new SparkMax(CanID, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
                this.configSPX = new SparkMaxConfig();
                break;
            case TFX:
                this.motorTFX = new TalonFX(CanID);
                this.configTFX = new TalonFXConfiguration();
                this.slot0TFX = new Slot0Configs();
                this.motorTFX.getConfigurator().setPosition(0);
                break;
            case None:
                System.err.println("Motor initialized with None type with CanID " + this.CanID);
        }

        this.motorConfig = new MotorConfig();
        this.applyConfig();
    }

    /**
     * Sets the duty cycle of the motor
     * 
     * @param dutyCycle from -1.0 to 1.0
     */
    public void dc(double dutyCycle) {
        switch (this.type) {
            case SPX:
                this.motorSPX.set(dutyCycle);
                break;
            case TFX:
                this.motorTFX.set(dutyCycle);
                break;
            case None:
                System.err.println("tried to set dc on None motor with CanID " + this.CanID);
        }
    }

    /**
     * Gets the duty cycle of the motor
     * 
     * @return the duty cycle from -1.0 to 1.0
     */
    public double dc() {
        switch (this.type) {
            case SPX:
                return this.motorSPX.get();
            case TFX:
                return this.motorTFX.get();
            default:
                return 0.0;
        }
    }

    /**
     * Sets the position of the motor without a feedforward
     * 
     * @param pos the position to set to in whatever unit is being used, usually
     *            motor rotations
     */
    public void pos(double pos) {
        switch (this.type) {
            case SPX:
                motorSPX.getClosedLoopController().setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
                break;
            case TFX:
                PositionDutyCycle controlRequest = new PositionDutyCycle(pos);
                controlRequest.FeedForward = 0;
                motorTFX.setControl(controlRequest);
                break;
            case None:
                System.err.println("tried to set pos on None motor with CanID " + this.CanID);
        }
    }

    /**
     * Sets the position of the motor with a feedforward
     * 
     * @param pos the position to set to in whatever unit is being used, usually
     *            motor rotations
     * @param ff  the feedforward
     */
    public void pos(double pos, double ff) {
        switch (this.type) {
            case SPX:
                motorSPX.getClosedLoopController().setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
                break;
            case TFX:
                PositionDutyCycle controlRequest = new PositionDutyCycle(pos);
                controlRequest.FeedForward = ff;
                motorTFX.setControl(controlRequest);
                break;
            case None:
                System.err.println("tried to set pos on None motor with CanID " + this.CanID);
        }
    }

    /**
     * Gets the position of the motor
     * 
     * @return the pos
     */
    public double pos() {
        switch (this.type) {
            case SPX:
                return this.motorSPX.getEncoder().getPosition();
            case TFX:
                return this.motorTFX.getRotorPosition().getValueAsDouble();
            default:
                return 0.0;
        }
    }

    /**
     * tells the motor that wherever it is is the value passed to it
     * 
     * @param position the pos to set the encoder value to
     */
    public void resetPos(double position) {
        switch (this.type) {
            case SPX:
                this.motorSPX.getEncoder().setPosition(position);
            case TFX:
                this.motorTFX.setPosition(position);
            default:
                System.err.println("tried to reset pos on None motor with CanID " + this.CanID);
        }
    }

    /**
     * gives current rotor velocity in RpS
     * @return the motor velocity
     */
    public double vel() {
        switch (this.type) {
            case SPX:
                return this.motorSPX.getEncoder().getVelocity();
            case TFX:
                return this.motorTFX.getVelocity().getValueAsDouble();
            default:
                return 0.0;
        }
    }

    /**
     * gets if there are currently any faults oon the motor
     * @return
     */
    public boolean isFault() {
        switch (this.type) {
            case SPX:
                return (short) 0 != motorSPX.getFaults().rawBits;
            case TFX:
                return 0 != motorTFX.getFaultField().getValueAsDouble();
            default:
                return false;
        }
    }

    public double getTemp() {
        switch (this.type) {
            case SPX:
                return motorSPX.getMotorTemperature();
            case TFX:
                return motorTFX.getDeviceTemp().getValueAsDouble();
            default:
                return 0.0;
        }
    }

    /**
     * Sets the PID of the motor
     * 
     * @param p proportional
     * @param i integral
     * @param d derivative
     */
    public void pid(double p, double i, double d) {
        switch (type) {
            case SPX:
                configSPX.closedLoop.pid(p, i, d);
                motorSPX.configure(configSPX, ResetMode.kNoResetSafeParameters,
                        PersistMode.kPersistParameters);
                break;
            case TFX:
                this.slot0TFX.kP = p;
                this.slot0TFX.kI = i;
                this.slot0TFX.kD = d;
                this.motorTFX.getConfigurator().apply(this.slot0TFX);
                break;
            case None:
                System.err.println("tried to set pid on None motor with CanID " + this.CanID);
        }
    }

    /**
     * set it to brake or coast, doesn't cause performance hit if the value isn't different
     * or if it is a TolonFX Motor, as TFX has a way of setting to idleMode
     * without pushing the configs, allowing better performance
     * 
     * @param brake true for brake, false for coast
     */
    public void setBrake(boolean brake) {
        if (brake != this.motorConfig.brake) {
            switch (type) {
                case SPX:
                    this.motorConfig.brake = brake;
                    this.applyConfig();
                    break;
                case TFX:
                    this.motorConfig.brake = brake;
                    this.motorTFX.setNeutralMode((brake) ? NeutralModeValue.Brake:NeutralModeValue.Coast);
                    break;
                case None:
                    System.err.println("tried to set brake mode on None motor with CanID " + this.CanID);
            }
        }
    }

    public void applyConfig() {
        switch (this.type) {
            case SPX:
                this.configSPX.inverted(this.motorConfig.direction == RotationDir.Clockwise);
                this.configSPX.idleMode((this.motorConfig.brake) ? IdleMode.kBrake : IdleMode.kCoast);
                this.motorSPX.configure(configSPX, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                break;
            case TFX:
                this.configTFX.MotorOutput.PeakForwardDutyCycle = this.motorConfig.peakForwardDC;
                this.configTFX.MotorOutput.PeakReverseDutyCycle = this.motorConfig.peakReverseDC;
                this.configTFX.MotorOutput.Inverted = (this.motorConfig.direction == RotationDir.Clockwise)
                        ? InvertedValue.Clockwise_Positive
                        : InvertedValue.CounterClockwise_Positive;
                this.configTFX.MotorOutput.NeutralMode = (this.motorConfig.brake) ? NeutralModeValue.Brake
                        : NeutralModeValue.Coast;
                this.configTFX.HardwareLimitSwitch.ForwardLimitEnable = this.motorConfig.forwardLimitSwitchEnabled;
                this.configTFX.HardwareLimitSwitch.ReverseLimitEnable = this.motorConfig.reverseLimitSwitchEnabled;
                this.motorTFX.getConfigurator().apply(configTFX);
                break;
            case None:
                System.err.println("tried to apply motor config on None motor with CanID " + this.CanID);
        }
    }

    public void applyTalonFxConfig(TalonFXConfiguration config) {
        // Apply the configuration passed in
        this.motorTFX.getConfigurator().apply(config); 
    }
}