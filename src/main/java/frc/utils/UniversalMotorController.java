package frc.utils;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

/**
 * Fat helper class to standardize motor control interfaces
 * for both TalonFX and SparkMax motor controllers.
 */
public class UniversalMotorController {
    public class Encoder {
        private final ControllerType type;
        private final RelativeEncoder sparkEncoder; // Only used if type is SPARK_MAX
        private final TalonFX talonFX; // Only used if type is TALON_FX
        
    
        public Encoder(TalonFX talonFX) {
            this.sparkEncoder = null;
            this.type = ControllerType.TALON_FX;
            this.talonFX = talonFX;
        }
        public Encoder(RelativeEncoder sparkEncoder) {
            this.talonFX = null;
            this.type = ControllerType.SPARK_MAX;
            this.sparkEncoder = sparkEncoder;
        }
        
        public double getPosition() {
            if (type == ControllerType.TALON_FX) {
                return talonFX.getPosition().getValueAsDouble();
            } else {
                return sparkEncoder.getPosition();
            }
        }
        
        public double getVelocity() {
            if (type == ControllerType.TALON_FX) {
                return talonFX.getVelocity().getValueAsDouble();
            } else {
                return sparkEncoder.getVelocity();
            }
        }
        public void setPosition(double position) {
            if (type == ControllerType.TALON_FX) {
                talonFX.setPosition(position);
            } else {
                sparkEncoder.setPosition(position);
            }
        }
    }
    public enum ControllerType {
        TALON_FX,
        SPARK_MAX
    }
    private Encoder encoder;
    private final ControllerType motorController;
    private TalonFX talonFX;
    private SparkMax sparkMax;
    private RelativeEncoder sparkEncoder; // fallback for direct access
    private SparkClosedLoopController sparkController;
    
    private VoltageOut voltageRequest;
    private VelocityVoltage velocityRequest;
    
    /**
     * Creates a new UniversalMotorController based on MotorType (or rather based on Motor Controller type).
     * @param canId The CAN ID of the motor
     * @param motorController The type of motor controller (TALON_FX or SPARK_MAX)
     */
    public UniversalMotorController(int canId, ControllerType motorController, SparkLowLevel.MotorType sparkMotorType) {
        switch (motorController) {
            case TALON_FX:
                this.motorController = ControllerType.TALON_FX;
                this.talonFX = new TalonFX(canId);
                this.voltageRequest = new VoltageOut(0);
                this.velocityRequest = new VelocityVoltage(0);
                break;
            case SPARK_MAX:
                this.motorController = ControllerType.SPARK_MAX;
                this.sparkMax = new SparkMax(canId, sparkMotorType);
                this.sparkEncoder = sparkMax.getEncoder();
                this.encoder = new Encoder(sparkEncoder);
                this.sparkController = sparkMax.getClosedLoopController();
                break;
            default:
                throw new IllegalArgumentException("Unsupported motor type: " + motorController);
        }
    }
    /**
     * Creates a new UniversalMotorController with either a TalonFX or brushless mode SparkMax.
     * @param canId The CAN ID of the motor
     * @param motorController The type of motor controller (TALON_FX or SPARK_MAX)
     */
    public UniversalMotorController(int canId, ControllerType motorController) {
        this(canId, motorController, SparkLowLevel.MotorType.kBrushless);
    }
    public Encoder getEncoder() {
        return this.encoder;
    }
    /**
     * Drop-in replacement for SparkMAX constructor
     * @param canId CAN ID of the motor controller
     * @param sparkMotorType The type of Spark motor (brushed or brushless) 
     */
    public UniversalMotorController(int canId, SparkLowLevel.MotorType sparkMotorType) {
        this(canId, ControllerType.SPARK_MAX, sparkMotorType);
    }
    /**
     * Sets the motor output voltage.
     * @param voltage The voltage to apply
     */
    public void setVoltage(Voltage voltage) {
        if (motorController == ControllerType.TALON_FX) {
            talonFX.setControl(voltageRequest.withOutput(voltage));
        } else {
            sparkMax.setVoltage(voltage.in(Volts));
        }
    }
    
    /**
     * Sets the motor output as a percentage (-1.0 to 1.0).
     * @param percent The percentage output
     */
    public void setPercent(double percent) {
        if (motorController == ControllerType.TALON_FX) {
            talonFX.set(percent);
        } else {
            sparkMax.set(percent);
        }
    }
    
    /**
     * Sets the motor to run at a target velocity using closed-loop control.
     * @param velocity The target velocity
     */
    public void setVelocity(AngularVelocity velocity) {
        if (motorController == ControllerType.TALON_FX) {
            talonFX.setControl(velocityRequest.withVelocity(velocity));
        } else {
            sparkController.setSetpoint(
                velocity.in(RPM),
                ControlType.kVelocity,
                com.revrobotics.spark.ClosedLoopSlot.kSlot0,
                0.0,
                SparkClosedLoopController.ArbFFUnits.kVoltage
            );
        }
    }
    
    /**
     * Sets the motor to run at a target velocity with arbitrary feedforward.
     * @param velocity The target velocity
     * @param arbFF The arbitrary feedforward in volts
     */
    public void setVelocity(AngularVelocity velocity, double arbFF) {
        if (motorController == ControllerType.TALON_FX) {
            talonFX.setControl(velocityRequest.withVelocity(velocity).withFeedForward(arbFF));
        } else {
            sparkController.setSetpoint(
                velocity.in(RPM),
                ControlType.kVelocity,
                com.revrobotics.spark.ClosedLoopSlot.kSlot0,
                arbFF,
                SparkClosedLoopController.ArbFFUnits.kVoltage
            );
        }
    }
    
    /**
     * Stops the motor.
     */
    public void stopMotor() {
        if (motorController == ControllerType.TALON_FX) {
            talonFX.stopMotor();
        } else {
            sparkMax.stopMotor();
        }
    }
    
    /**
     * Gets the motor's output voltage.
     * @return The output voltage
     */
    public Voltage getMotorVoltage() {
        if (motorController == ControllerType.TALON_FX) {
            return talonFX.getMotorVoltage().getValue();
        } else {
            return Volts.of(sparkMax.getAppliedOutput() * sparkMax.getBusVoltage());
        }
    }
    
    /**
     * Gets the motor's supply current.
     * @return The supply current
     */
    public Current getSupplyCurrent() {
        if (motorController == ControllerType.TALON_FX) {
            return talonFX.getSupplyCurrent().getValue();
        } else {
            return Amps.of(sparkMax.getOutputCurrent());
        }
    }
    
    /**
     * Gets the motor's temperature.
     * @return The motor temperature
     */
    public Temperature getTemperature() {
        if (motorController == ControllerType.TALON_FX) {
            return talonFX.getDeviceTemp().getValue();
        } else {
            return Celsius.of(sparkMax.getMotorTemperature());
        }
    }
    
    /**
     * Checks if the motor is connected.
     */
    public boolean isConnected() {
        if (motorController == ControllerType.TALON_FX) {
            return talonFX.isConnected();
        } else {
            // stupid sparkmax doesn't have a direct isConnected method so use extremely cheap workaround
            return sparkMax.getFirmwareString() != null && !sparkMax.getFirmwareString().isEmpty();
        }
    }
    
    /**
     * Sets the inverted state of the motor.
     */
    public void setInverted(boolean inverted) {
        if (motorController == ControllerType.TALON_FX) {
            com.ctre.phoenix6.configs.MotorOutputConfigs config = new com.ctre.phoenix6.configs.MotorOutputConfigs();
            config.Inverted = inverted ? 
                com.ctre.phoenix6.signals.InvertedValue.Clockwise_Positive : 
                com.ctre.phoenix6.signals.InvertedValue.CounterClockwise_Positive;
            talonFX.getConfigurator().apply(config);
        } else {
            SparkMaxConfig config = new SparkMaxConfig();
            config.inverted(inverted);
            sparkMax.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, 
                              com.revrobotics.PersistMode.kNoPersistParameters);
        }
    }
    
    /**
     * Configures PID constants for the motor controller.
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void configurePID(double kP, double kI, double kD) {
        if (motorController == ControllerType.TALON_FX) {
            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = kP;
            slot0.kI = kI;
            slot0.kD = kD;
            talonFX.getConfigurator().apply(slot0);
        } else {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.pid(kP, kI, kD);
            sparkMax.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, 
                              com.revrobotics.PersistMode.kNoPersistParameters);
        }
    }
    
    /**
     * Configures feedforward constants for the motor controller.
     * @param kS Static gain
     * @param kV Velocity gain
     * @param kA Acceleration gain
     */
    public void configureFeedforward(double kS, double kV, double kA) {
        if (motorController == ControllerType.TALON_FX) {
            Slot0Configs slot0 = new Slot0Configs();
            slot0.kS = kS;
            slot0.kV = kV;
            slot0.kA = kA;
            talonFX.getConfigurator().apply(slot0);
        } else {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.feedForward.kS(kS);
            config.closedLoop.feedForward.kV(kV);
            config.closedLoop.feedForward.kA(kA);
            sparkMax.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, 
                              com.revrobotics.PersistMode.kNoPersistParameters);
        }
    }
    
    /**
     * Configures both PID and feedforward constants.
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     * @param kS Static gain
     * @param kV Velocity gain
     * @param kA Acceleration gain
     */
    public void configureClosedLoop(double kP, double kI, double kD, double kS, double kV, double kA) {
        if (motorController == ControllerType.TALON_FX) {
            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = kP;
            slot0.kI = kI;
            slot0.kD = kD;
            slot0.kS = kS;
            slot0.kV = kV;
            slot0.kA = kA;
            talonFX.getConfigurator().apply(slot0);
        } else {
            SparkMaxConfig config = new SparkMaxConfig();
            config.closedLoop.pid(kP, kI, kD);
            config.closedLoop.feedForward.kS(kS);
            config.closedLoop.feedForward.kV(kV);
            config.closedLoop.feedForward.kA(kA);
            sparkMax.configure(config, com.revrobotics.ResetMode.kNoResetSafeParameters, 
                              com.revrobotics.PersistMode.kNoPersistParameters);
        }
    }
    
    /**
     * Gets the underlying motor controller type.
     * @return The motor controller type (TALON_FX or SPARK_MAX)
     */
    public ControllerType getMotorController() {
        return motorController;
    }
    
    /**
     * Gets the underlying TalonFX motor controller (if applicable).
     * @return The TalonFX instance, or null if this is a SparkMax
     */
    public TalonFX getTalonFX() {
        return talonFX;
    }
    
    /**
     * Gets the underlying SparkMax motor controller (if applicable).
     * @return The SparkMax instance, or null if this is a TalonFX
     */
    public SparkMax getSparkMax() {
        return sparkMax;
    }
}
