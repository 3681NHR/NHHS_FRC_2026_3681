package frc.robot.subsystems.swerve.module;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.DriveConstants.ODOMETRY_FREQ;
import static frc.utils.SparkUtil.sparkStickyFault;
import static frc.utils.SparkUtil.tryUntilOk;

import java.util.Queue;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import frc.utils.motorWrappers.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.utils.motorWrappers.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.DriveConstants.Module;
import frc.utils.PhoenixOdometryThread;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;

/**
 * Module IO implementation for Spark Flex drive motor controller, Spark Max
 * turn motor controller,
 * and duty cycle absolute encoder.
 */
public class ModuleIOCrackingSpark implements ModuleIO {

    // Hardware objects
    private final TalonFX driveTalon;
    private final SparkBase turnSpark;
    private final AbsoluteEncoder turnEncoder;

    private final VoltageOut driveOpenLoopOut = new VoltageOut(0);
    // Closed loop controllers
    private final VelocityVoltage driveController = new VelocityVoltage(RPM.of(0));
    private final ProfiledPID turnPID = new ProfiledPID(Module.TURN_PID);
    private final SimpleFF turnFF = new SimpleFF(Module.TURN_FF);

    // Queue inputs from odometry thread
    private final Queue<Double> timestampQueue;
    private final Queue<Double> drivePositionQueue;
    private final Queue<Double> turnPositionQueue;

    // Connection debouncers
    private final Debouncer driveConnectedDebounce = new Debouncer(0.5);
    private final Debouncer turnConnectedDebounce = new Debouncer(0.5);

    private Angle turnGoal = Radians.of(0.0);
    private AngularVelocity driveGoal = RPM.of(0.0);

    private boolean driveClosedLoop = true;
    private boolean turnClosedLoop = true;

    private StatusSignal<Angle> drivePosition;
    private StatusSignal<AngularVelocity> driveVelocity;
    private StatusSignal<Temperature> driveTemp;
    private StatusSignal<Current> driveSupplyCurrent;
    private StatusSignal<Voltage> driveAppliedVolts;
    
    private Angle turnPosition = Radians.of(0.0);
    private AngularVelocity turnVelocity = RadiansPerSecond.of(0.0);

    private Voltage driveOpenLoopVout = Volts.of(0);
    private Voltage turnOpenLoopVout = Volts.of(0);

	public ModuleIOCrackingSpark(int io) {
        driveTalon = new TalonFX(
                switch (io) {
                    case 0 -> Module.FL_DRIVE_ID;
                    case 1 -> Module.FR_DRIVE_ID;
                    case 2 -> Module.BL_DRIVE_ID;
                    case 3 -> Module.BR_DRIVE_ID;
                    default -> 0;
                });
        turnSpark = new SparkMax(
                switch (io) {
                    case 0 -> Module.FL_TURN_ID;
                    case 1 -> Module.FR_TURN_ID;
                    case 2 -> Module.BL_TURN_ID;
                    case 3 -> Module.BR_TURN_ID;
                    default -> 0;
                },
                MotorType.kBrushless);
        turnEncoder = turnSpark.getAbsoluteEncoder();

        // Configure drive motor
        var driveConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(Module.DRIVE_INVERT ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
                        )
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(Module.DRIVE_MAX_CURRENT)
                        .withStatorCurrentLimit(Module.DRIVE_SLIP_CURRENT)
                        )
                .withSlot0(new Slot0Configs()
                        .withKP(Module.DRIVE_PID.kP)
                        .withKI(Module.DRIVE_PID.kI)
                        .withKD(Module.DRIVE_PID.kD)
                        .withKS(Module.DRIVE_FF.kS)
                        .withKV(Module.DRIVE_FF.kV)
                        .withKA(Module.DRIVE_FF.kA)
                        .withKG(0)
                        )
                .withFeedback(new FeedbackConfigs()
                        .withSensorToMechanismRatio(Module.DRIVE_REDUCTION)
                        .withRotorToSensorRatio(1)
                        );
        driveTalon.getConfigurator().apply(driveConfig);

        Module.DRIVE_PID.withCallback(() -> {
            driveTalon.getConfigurator().apply(new Slot0Configs()
                        .withKP(Module.DRIVE_PID.kP)
                        .withKI(Module.DRIVE_PID.kI)
                        .withKD(Module.DRIVE_PID.kD)
                        .withKS(Module.DRIVE_FF.kS)
                        .withKV(Module.DRIVE_FF.kV)
                        .withKA(Module.DRIVE_FF.kA)
                        .withKG(0)
                        );
        });
        Module.DRIVE_FF.withCallback(() -> {
            driveTalon.getConfigurator().apply(new Slot0Configs()
                        .withKP(Module.DRIVE_PID.kP)
                        .withKI(Module.DRIVE_PID.kI)
                        .withKD(Module.DRIVE_PID.kD)
                        .withKS(Module.DRIVE_FF.kS)
                        .withKV(Module.DRIVE_FF.kV)
                        .withKA(Module.DRIVE_FF.kA)
                        .withKG(0)
                        );
        });

        driveAppliedVolts = driveTalon.getMotorVoltage();
        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveTemp = driveTalon.getDeviceTemp();
        driveSupplyCurrent = driveTalon.getSupplyCurrent();

        // Configure turn motor
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(Module.TURN_INVERT)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit((int) Module.TURN_CURRENT_LIM.in(Amps))
                .voltageCompensation(12.0);
        turnConfig.absoluteEncoder
                .inverted(Module.TURN_ENCODER_INVERT)
                .positionConversionFactor(Module.TURN_ENCODER_POS_FACTOR)
                .velocityConversionFactor(Module.TURN_ENCODER_VEL_FACTOR)
                .averageDepth(8);
        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        turnConfig.signals
                .absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderPositionPeriodMs((int) (1000.0 / ODOMETRY_FREQ.in(Hertz)))
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

        turnPID.enableContinuousInput(Module.TURN_MIN_POS.in(Radians), Module.TURN_MAX_POS.in(Radians));
        // Create odometry queues
        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition.clone());
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(turnEncoder::getPosition);

        
        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.ODOMETRY_FREQ.in(Hertz), drivePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveTemp,
            driveSupplyCurrent,
            driveTalon.getClosedLoopReference()
            );
        ParentDevice.optimizeBusUtilizationForAll(driveTalon);

        driveTalon.setPosition(0);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        turnPosition = Radians.of(turnEncoder.getPosition());
        turnVelocity = RPM.of(turnEncoder.getVelocity());

        // Update drive inputs
        inputs.driveTemp = driveTemp.refresh().getValue();
        inputs.drivePosition = drivePosition.refresh().getValue();
        inputs.driveVelocity = driveVelocity.refresh().getValue();
        inputs.driveAppliedVolts = driveAppliedVolts.refresh().getValue();
        inputs.driveConnected = driveConnectedDebounce.calculate(driveTalon.isConnected());
        inputs.driveGoal = driveGoal;
        inputs.driveSetpoint = Rotations.per(Second).of(driveTalon.getClosedLoopReference().getValue());
        inputs.driveCurrent = driveSupplyCurrent.refresh().getValue();
        inputs.driveOpenLoop = !driveClosedLoop;

        // Update turn inputs
        sparkStickyFault = false;

        inputs.turnPosition = turnPosition;
        inputs.turnVelocity = turnVelocity;
        inputs.turnAppliedVolts = Volts.of(turnSpark.getAppliedOutput() * turnSpark.getBusVoltage());
        inputs.turnCurrent = Amps.of(turnSpark.getOutputCurrent());
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault);
        inputs.turnGoal = turnGoal;
        inputs.turnSetpoint = Radians.of(turnPID.getSetpoint().position);
        inputs.turnTemp = Temperature.ofBaseUnits(turnSpark.getMotorTemperature(), Celsius);
        inputs.turnCurrent = Amps.of(turnSpark.getOutputCurrent());
        inputs.turnOpenLoop = !turnClosedLoop;
        
        // Update odometry inputs
        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble((Double value) -> Units.rotationsToRadians(value)).toArray();
        
        inputs.odometryTurnPositionsRad = turnPositionQueue.stream()
                .mapToDouble((Double value) -> value)
                .toArray();
        timestampQueue.clear();
        drivePositionQueue.clear();
        turnPositionQueue.clear();
    }
    
    public double getDriveOffsetVelocity() {
        return turnEncoder.getVelocity() * Module.DRIVE_OFFSET_VEL_FACTOR;
    }

    @Override
    public void setDriveOpenLoop(Voltage output) {
        driveOpenLoopVout = output;
        driveClosedLoop = false;
        driveTalon.setControl(driveOpenLoopOut.withOutput(driveOpenLoopVout));

    }

    @Override
    public void setTurnOpenLoop(Voltage output) {
        turnOpenLoopVout = output;
        turnClosedLoop = false;
        turnSpark.setVoltage(turnOpenLoopVout.in(Volts));
    }

    @Override
    public void setDriveVelocity(AngularVelocity velocity) {
        driveGoal = velocity;
        driveClosedLoop = true;
        driveTalon.setControl(driveController.withVelocity(driveGoal.plus(RadiansPerSecond.of(getDriveOffsetVelocity()))));

    }

    @Override
    public void setTurnPosition(Angle rotation) {
        turnGoal = Radians.of(MathUtil.inputModulus(rotation.in(Radians), Module.TURN_MIN_POS.in(Radians), Module.TURN_MAX_POS.in(Radians)));
        turnClosedLoop = true;
        // First, advance the profiled PID to the new goal to update its internal setpoint
        double pidVolts = turnPID.calculate(turnEncoder.getPosition(), turnGoal.in(Radians));
        // Then compute feedforward from the updated setpoint velocity
        double ffVolts = turnFF.calculate(turnPID.getSetpoint().velocity);
        turnSpark.setVoltage(ffVolts + pidVolts);
    }
}
