package frc.robot.subsystems.swerve.module

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.ParentDevice
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.revrobotics.AbsoluteEncoder
import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.FeedbackSensor
import com.revrobotics.spark.SparkBase
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import frc.robot.constants.DriveConstants
import frc.robot.constants.DriveConstants.Module
import frc.utils.PhoenixOdometryThread
import frc.utils.SparkUtil.sparkStickyFault
import frc.utils.SparkUtil.tryUntilOk
import frc.utils.controlWrappers.ProfiledPID
import frc.utils.controlWrappers.SimpleFF
import frc.utils.motorWrappers.SparkMax
import frc.utils.motorWrappers.TalonFX
import java.util.Queue

class ModuleIOCrackingSpark(io: Int) : ModuleIO {

    private val driveTalon: TalonFX
    private val turnSpark: SparkBase
    private val turnEncoder: AbsoluteEncoder

    private val driveOpenLoopOut = VoltageOut(0.0)
    private val driveController = VelocityVoltage(RPM.of(0.0))
    private val turnPID = ProfiledPID(Module.TURN_PID)
    private val turnFF = SimpleFF(Module.TURN_FF)

    private val timestampQueue: Queue<Double>
    private val drivePositionQueue: Queue<Double>
    private val turnPositionQueue: Queue<Double>

    private val driveConnectedDebounce = Debouncer(0.5)
    private val turnConnectedDebounce = Debouncer(0.5)

    private var turnGoal: Angle = Radians.of(0.0)
    private var driveGoal: AngularVelocity = RPM.of(0.0)

    private var driveClosedLoop: Boolean = true
    private var turnClosedLoop: Boolean = true

    private val drivePosition: StatusSignal<Angle>
    private val driveVelocity: StatusSignal<AngularVelocity>
    private val driveTemp: StatusSignal<Temperature>
    private val driveSupplyCurrent: StatusSignal<Current>
    private val driveAppliedVolts: StatusSignal<Voltage>

    private var turnVelocity: AngularVelocity = RadiansPerSecond.of(0.0)

    private var driveOpenLoopVout: Voltage = Volts.of(0.0)
    private var turnOpenLoopVout: Voltage = Volts.of(0.0)

    init {
        driveTalon = TalonFX(
            when (io) {
                0 -> Module.FL_DRIVE_ID
                1 -> Module.FR_DRIVE_ID
                2 -> Module.BL_DRIVE_ID
                3 -> Module.BR_DRIVE_ID
                else -> 0
            }
        )
        turnSpark = SparkMax(
            when (io) {
                0 -> Module.FL_TURN_ID
                1 -> Module.FR_TURN_ID
                2 -> Module.BL_TURN_ID
                3 -> Module.BR_TURN_ID
                else -> 0
            },
            MotorType.kBrushless
        )
        turnEncoder = turnSpark.absoluteEncoder

        val driveConfig = TalonFXConfiguration()
            .withMotorOutput(
                MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(if (Module.DRIVE_INVERT) InvertedValue.CounterClockwise_Positive else InvertedValue.Clockwise_Positive)
            )
            .withCurrentLimits(
                CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(Module.DRIVE_MAX_CURRENT)
                    .withStatorCurrentLimit(Module.DRIVE_SLIP_CURRENT)
            )
            .withSlot0(
                Slot0Configs()
                    .withKP(Module.DRIVE_PID.kP)
                    .withKI(Module.DRIVE_PID.kI)
                    .withKD(Module.DRIVE_PID.kD)
                    .withKS(Module.DRIVE_FF.kS)
                    .withKV(Module.DRIVE_FF.kV)
                    .withKA(Module.DRIVE_FF.kA)
                    .withKG(0.0)
            )
            .withFeedback(
                FeedbackConfigs()
                    .withSensorToMechanismRatio(Module.DRIVE_REDUCTION)
                    .withRotorToSensorRatio(1.0)
            )
            .withFeedback(FeedbackConfigs().withSensorToMechanismRatio(6.75))
        driveTalon.configurator.apply(driveConfig)

        Module.DRIVE_PID.withCallback(Runnable {
            driveTalon.configurator.apply(
                Slot0Configs()
                    .withKP(Module.DRIVE_PID.kP)
                    .withKI(Module.DRIVE_PID.kI)
                    .withKD(Module.DRIVE_PID.kD)
                    .withKS(Module.DRIVE_FF.kS)
                    .withKV(Module.DRIVE_FF.kV)
                    .withKA(Module.DRIVE_FF.kA)
                    .withKG(0.0)
            )
        })
        Module.DRIVE_FF.withCallback(Runnable {
            driveTalon.configurator.apply(
                Slot0Configs()
                    .withKP(Module.DRIVE_PID.kP)
                    .withKI(Module.DRIVE_PID.kI)
                    .withKD(Module.DRIVE_PID.kD)
                    .withKS(Module.DRIVE_FF.kS)
                    .withKV(Module.DRIVE_FF.kV)
                    .withKA(Module.DRIVE_FF.kA)
                    .withKG(0.0)
            )
        })

        driveAppliedVolts = driveTalon.motorVoltage
        drivePosition = driveTalon.position
        driveVelocity = driveTalon.velocity
        driveTemp = driveTalon.deviceTemp
        driveSupplyCurrent = driveTalon.supplyCurrent

        val turnConfig = SparkMaxConfig()
        turnConfig
            .inverted(Module.TURN_INVERT)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Module.TURN_CURRENT_LIM.`in`(Amps).toInt())
            .voltageCompensation(12.0)
        turnConfig.absoluteEncoder
            .inverted(Module.TURN_ENCODER_INVERT)
            .positionConversionFactor(Module.TURN_ENCODER_POS_FACTOR)
            .velocityConversionFactor(Module.TURN_ENCODER_VEL_FACTOR)
            .averageDepth(8)
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        turnConfig.signals
            .absoluteEncoderPositionAlwaysOn(true)
            .absoluteEncoderPositionPeriodMs((1000.0 / DriveConstants.ODOMETRY_FREQ.`in`(Hertz)).toInt())
            .absoluteEncoderVelocityAlwaysOn(true)
            .absoluteEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20)
        tryUntilOk(
            turnSpark,
            5
        ) { turnSpark.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters) }

        turnPID.enableContinuousInput(Module.TURN_MIN_POS.`in`(Radians), Module.TURN_MAX_POS.`in`(Radians))

        timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue()
        drivePositionQueue = PhoenixOdometryThread.getInstance().registerSignal(drivePosition.clone())
        turnPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(DoubleSupplierAdapter { turnEncoder.position })

        BaseStatusSignal.setUpdateFrequencyForAll(
            DriveConstants.ODOMETRY_FREQ.`in`(Hertz), drivePosition
        )
        BaseStatusSignal.setUpdateFrequencyForAll(
            50.0,
            driveVelocity,
            driveAppliedVolts,
            driveTemp,
            driveSupplyCurrent,
            driveTalon.closedLoopReference
        )
        ParentDevice.optimizeBusUtilizationForAll(driveTalon)

        driveTalon.setPosition(0.0)
    }

    override fun updateInputs(inputs: ModuleIO.ModuleIOInputs) {
        val turnPosition = Radians.of(turnEncoder.position)
        turnVelocity = RPM.of(turnEncoder.velocity)

        inputs.driveTemp = driveTemp.refresh().value
        inputs.drivePosition = drivePosition.refresh().value
        inputs.driveVelocity = driveVelocity.refresh().value
        inputs.driveAppliedVolts = driveAppliedVolts.refresh().value
        inputs.driveConnected = driveConnectedDebounce.calculate(driveTalon.isConnected)
        inputs.driveGoal = driveGoal
        inputs.driveSetpoint = Rotations.per(Second).of(driveTalon.closedLoopReference.value)
        inputs.driveCurrent = driveSupplyCurrent.refresh().value
        inputs.driveOpenLoop = !driveClosedLoop

        sparkStickyFault = false

        inputs.turnPosition = turnPosition
        inputs.turnVelocity = turnVelocity
        inputs.turnAppliedVolts = Volts.of(turnSpark.appliedOutput * turnSpark.busVoltage)
        inputs.turnCurrent = Amps.of(turnSpark.outputCurrent)
        inputs.turnConnected = turnConnectedDebounce.calculate(!sparkStickyFault)
        inputs.turnGoal = turnGoal
        inputs.turnSetpoint = Radians.of(turnPID.setpoint.position)
        inputs.turnTemp = Temperature.ofBaseUnits(turnSpark.motorTemperature, Celsius)
        inputs.turnCurrent = Amps.of(turnSpark.outputCurrent)
        inputs.turnOpenLoop = !turnClosedLoop

        inputs.odometryTimestamps = timestampQueue.stream().mapToDouble { it }.toArray()
        inputs.odometryDrivePositionsRad = drivePositionQueue.stream().mapToDouble { Units.rotationsToRadians(it) }.toArray()
        inputs.odometryTurnPositionsRad = turnPositionQueue.stream().mapToDouble { it }.toArray()
        timestampQueue.clear()
        drivePositionQueue.clear()
        turnPositionQueue.clear()
    }

    fun getDriveOffsetVelocity(): Double {
        return turnEncoder.velocity * Module.DRIVE_OFFSET_VEL_FACTOR
    }

    override fun setDriveOpenLoop(output: Voltage) {
        driveOpenLoopVout = output
        driveClosedLoop = false
        driveTalon.setControl(driveOpenLoopOut.withOutput(driveOpenLoopVout))
    }

    override fun setTurnOpenLoop(output: Voltage) {
        turnOpenLoopVout = output
        turnClosedLoop = false
        turnSpark.setVoltage(turnOpenLoopVout.`in`(Volts))
    }

    override fun setDriveVelocity(velocity: AngularVelocity) {
        driveGoal = velocity
        driveClosedLoop = true
        driveTalon.setControl(driveController.withVelocity(driveGoal.plus(RadiansPerSecond.of(getDriveOffsetVelocity()))))
    }

    override fun setTurnPosition(rotation: Angle) {
        turnGoal = Radians.of(MathUtil.inputModulus(rotation.`in`(Radians), Module.TURN_MIN_POS.`in`(Radians), Module.TURN_MAX_POS.`in`(Radians)))
        turnClosedLoop = true
        val pidVolts = turnPID.calculate(turnEncoder.position, turnGoal.`in`(Radians))
        val ffVolts = turnFF.calculate(turnPID.setpoint.velocity)
        turnSpark.setVoltage(ffVolts + pidVolts)
    }

    // Adapter to convert lambda to DoubleSupplier for PhoenixOdometryThread
    private class DoubleSupplierAdapter(private val fn: () -> Double) : java.util.function.DoubleSupplier {
        override fun getAsDouble(): Double = fn()
    }
}
