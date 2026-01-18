package frc.robot.subsystems.swerve.module;

import static frc.robot.constants.DriveConstants.ODOMETRY_FREQ;
import static frc.utils.SparkUtil.ifOk;
import static frc.utils.SparkUtil.sparkStickyFault;
import static frc.utils.SparkUtil.tryUntilOk;

import java.util.Queue;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.DriveConstants.module;
import frc.utils.SparkOdometryThread;
import frc.utils.Symphony;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max
 * turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOCrackingSpark implements ModuleIO {

    // Hardware objects
    private final CoreTalonFX driveTalon;
    private final SparkBase turnSpark;
    private final AbsoluteEncoder turnEncoder;

    // Closed loop controllers
    private final VelocityVoltage driveController;
    private final ProfiledPID turnPID = new ProfiledPID(module.TURN_PID);
    private final SimpleFF turnFF = new SimpleFF(module.TURN_FF);
    private final SimpleFF driveFF = new SimpleFF(module.DRIVE_FF);

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    private double turnGoal = 0.0;
    private double driveGoal = 0.0;

    private boolean driveClosedLoop = true;
    private boolean turnClosedLoop = true;

    private double drivePositionRad = 0.0;
    private double turnPositionRad = 0.0;

    private double driveVelocityRadPerSecond = 0.0;
    private double turnVelocityRadPerSecond = 0.0;

	public ModuleIOCrackingSpark(int IO) {
        driveTalon = new TalonFX(
                switch (IO) {
                    case 0 -> module.FL_DRIVE_ID;
                    case 1 -> module.FR_DRIVE_ID;
                    case 2 -> module.BL_DRIVE_ID;
                    case 3 -> module.BR_DRIVE_ID;
                    default -> 0;
                });
        turnSpark = new SparkMax(
                switch (IO) {
                    case 0 -> module.FL_TURN_ID;
                    case 1 -> module.FR_TURN_ID;
                    case 2 -> module.BL_TURN_ID;
                    case 3 -> module.BR_TURN_ID;
                    default -> 0;
                },
                MotorType.kBrushless);
        turnEncoder = turnSpark.getAbsoluteEncoder();
        driveController = new VelocityVoltage(0).withSlot(0);

        // Configure drive motor
        var driveConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(module.DRIVE_INVERT ? InvertedValue.CounterClockwise_Positive
                                : InvertedValue.Clockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(module.DRIVE_MAX_CURRENT))
                .withSlot0(new Slot0Configs().withKP(module.DRIVE_PID.kP()).withKI(module.DRIVE_PID.kI())
                        .withKD(module.DRIVE_PID.kD()))
                .withFeedback(new FeedbackConfigs().withRotorToSensorRatio(module.DRIVE_REDUCTION));
        driveTalon.getConfigurator().apply(driveConfig);

        Symphony.getSymphony().registerInstrument(driveTalon);

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(module.TURN_INVERT)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(module.TURN_CURRENT_LIM)
                .voltageCompensation(12.0);
        turnConfig.absoluteEncoder
                .inverted(module.TURN_ENCODER_INVERT)
                .positionConversionFactor(module.TURN_ENCODER_POS_FACTOR)
                .velocityConversionFactor(module.TURN_ENCODER_VEL_FACTOR)
                .averageDepth(8);
        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        turnConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQ))
                .absoluteEncoderVelocityAlwaysOn(true)
                .absoluteEncoderVelocityPeriodMs(20)
                .appliedOutputPeriodMs(20)
                .busVoltagePeriodMs(20)
                .outputCurrentPeriodMs(20);
        tryUntilOk(
                turnSpark,
                5,
                () -> turnSpark.configure(
                        turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

        turnPID.enableContinuousInput(module.TURN_MIN_POS, module.TURN_MAX_POS);
        // Create odometry queues
        timestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = SparkOdometryThread.getInstance().registerSignal(() -> driveTalon.getPosition().getValueAsDouble());
        turnPositionQueue = SparkOdometryThread.getInstance().registerSignal(turnSpark, turnEncoder::getPosition);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        drivePositionRad = driveTalon.getPosition().getValueAsDouble() * module.DRIVE_ENCODER_POS_FACTOR;

        driveVelocityRadPerSecond = driveTalon.getVelocity().getValueAsDouble() * module.DRIVE_ENCODER_VEL_FACTOR;
        turnVelocityRadPerSecond = turnEncoder.getVelocity();

        inputs.turnTemp = turnSpark.getMotorTemperature();
        inputs.driveTemp = driveTalon.getDeviceTemp().getValueAsDouble();

        // Update drive inputs
        inputs.drivePositionRad = drivePositionRad;
        inputs.driveVelocityRadPerSec = driveVelocityRadPerSecond;
        inputs.driveAppliedVolts = driveTalon.getDutyCycle().getValueAsDouble()*driveTalon.getSupplyVoltage().getValueAsDouble();
        inputs.driveCurrentAmps = driveTalon.getSupplyCurrent().getValueAsDouble();
        inputs.driveConnected = driveConnectedDebounce.calculate(driveTalon.isConnected());

        // Update turn inputs
        sparkStickyFault = false;
        ifOk(
                turnSpark,
                () -> turnPositionRad,
                (value) -> inputs.turnPositionRad = value);
        ifOk(turnSpark, () -> turnVelocityRadPerSecond, (value) -> inputs.turnVelocityRadPerSec = value);
        ifOk(
                turnSpark,
                new DoubleSupplier[] { turnSpark::getAppliedOutput, turnSpark::getBusVoltage },
                (values) -> inputs.turnAppliedVolts = values[0] * values[1]);
        ifOk(turnSpark, turnSpark::getOutputCurrent, (value) -> inputs.turnCurrentAmps = value);
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);

        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryTurnPositionsRad = turnPositionQueue.stream()
                .mapToDouble((Double value) -> value)
                .toArray();
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();

        if (driveClosedLoop) {
            double ffVolts = driveFF.calculate(driveGoal);
            driveTalon.setControl(driveController.withVelocity(driveGoal/6.28 + getDriveOffsetVelocity()/6.28).withFeedForward(ffVolts));
        }
        if (turnClosedLoop) {
            double ffVolts = turnFF.calculate(turnPID.getSetpoint().velocity);
            turnPID.setGoal(turnGoal);
            turnSpark.setVoltage(ffVolts + turnPID.calculate(turnEncoder.getPosition()));
        }

    }
    public double getDriveOffsetVelocity() {
        return turnEncoder.getVelocity() * module.DRIVE_OFFSET_VEL_FACTOR;
    }

    @Override
    public void setDriveOpenLoop(double output) {
        driveTalon.setControl(new VoltageOut(output));
        driveClosedLoop = false;
    }

    @Override
    public void setTurnOpenLoop(double output) {
        turnSpark.setVoltage(output);
        turnClosedLoop = false;
    }

    @Override
    public void setDriveVelocity(double velocityRadPerSec) {
        driveGoal = velocityRadPerSec;
        driveClosedLoop = true;
    }

    @Override
    public void setTurnPosition(Rotation2d rotation) {
        turnGoal = MathUtil.inputModulus(rotation.getRadians(), module.TURN_MIN_POS, module.TURN_MAX_POS);
        turnClosedLoop = true;
    }
}
