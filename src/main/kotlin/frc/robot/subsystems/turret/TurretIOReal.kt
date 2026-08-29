package frc.robot.subsystems.turret

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotionMagicConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.controls.MotionMagicVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.robot.constants.TurretConstants.TURRET_ANGLE_FORWARD_LIM
import frc.robot.constants.TurretConstants.TURRET_ANGLE_REVERSE_LIM
import frc.robot.constants.TurretConstants.TURRET_CURRENT_LIM
import frc.robot.constants.TurretConstants.TURRET_ENCODER_1_GEAR_TEETH
import frc.robot.constants.TurretConstants.TURRET_ENCODER_1_ID
import frc.robot.constants.TurretConstants.TURRET_ENCODER_2_GEAR_TEETH
import frc.robot.constants.TurretConstants.TURRET_ENCODER_2_ID
import frc.robot.constants.TurretConstants.TURRET_FF_GAINS
import frc.robot.constants.TurretConstants.TURRET_MAIN_GEAR_TEETH
import frc.robot.constants.TurretConstants.TURRET_MOTOR_GEAR_TEETH
import frc.robot.constants.TurretConstants.TURRET_MOTOR_ID
import frc.robot.constants.TurretConstants.TURRET_MOTOR_INVERT
import frc.robot.constants.TurretConstants.TURRET_PID_GAINS
import frc.robot.constants.TurretConstants.TURRET_SETPOINT_TOLERANCE
import frc.utils.ExtraMath
import frc.utils.motorWrappers.TalonFX
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

class TurretIOReal : TurretIO {

    private val motor = TalonFX(TURRET_MOTOR_ID)
    private val e1 = CANcoder(TURRET_ENCODER_1_ID)
    private val e2 = CANcoder(TURRET_ENCODER_2_ID)

    private var openLoop = false
    private val closedLoopControl = MotionMagicVoltage(Radians.of(0.0))
    private val openLoopControl = VoltageOut(0.0)
    private var goal: Angle = Radians.of(0.0)

    private var tolerance: Angle = TURRET_SETPOINT_TOLERANCE

    private val motorDisconnect = Alert("Turret motor is disconnected!", Alert.AlertType.kError)
    private val e1Disconnect = Alert("Turret encoder 1 is disconnected!", Alert.AlertType.kError)
    private val e2Disconnect = Alert("Turret encoder 2 is disconnected!", Alert.AlertType.kError)

    private var homed = false

    private val resetTurretPos = LoggedNetworkBoolean("Debug/reset turret pos", false)

    init {
        val config = TalonFXConfiguration()

        config.MotionMagic = MotionMagicConfigs()
            .withMotionMagicAcceleration(TURRET_PID_GAINS.maxAccel)
            .withMotionMagicCruiseVelocity(TURRET_PID_GAINS.maxSpeed)

        config.Slot0 = Slot0Configs()
            .withKP(TURRET_PID_GAINS.kP)
            .withKI(TURRET_PID_GAINS.kI)
            .withKD(TURRET_PID_GAINS.kD)
            .withKS(0.0) // external ks is used
            .withKV(TURRET_FF_GAINS.kV)
            .withKA(TURRET_FF_GAINS.kA)

        config.SoftwareLimitSwitch = SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(TURRET_ANGLE_FORWARD_LIM)
            .withReverseSoftLimitThreshold(TURRET_ANGLE_REVERSE_LIM)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true)

        config.CurrentLimits = CurrentLimitsConfigs().withSupplyCurrentLimit(TURRET_CURRENT_LIM).withSupplyCurrentLimitEnable(true)

        config.MotorOutput = MotorOutputConfigs()
            .withInverted(if (TURRET_MOTOR_INVERT) InvertedValue.Clockwise_Positive else InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake)

        config.Feedback = FeedbackConfigs().withSensorToMechanismRatio(TURRET_MAIN_GEAR_TEETH / TURRET_MOTOR_GEAR_TEETH)

        config.SoftwareLimitSwitch = SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(TURRET_ANGLE_FORWARD_LIM)
            .withReverseSoftLimitEnable(true)
            .withReverseSoftLimitThreshold(TURRET_ANGLE_REVERSE_LIM)

        motor.configurator.apply(config)

        TURRET_PID_GAINS.withCallback {
            motor.configurator.apply(
                Slot0Configs()
                    .withKP(TURRET_PID_GAINS.kP)
                    .withKI(TURRET_PID_GAINS.kI)
                    .withKD(TURRET_PID_GAINS.kD)
            )
        }
        TURRET_FF_GAINS.withCallback {
            motor.configurator.apply(
                Slot0Configs()
                    .withKS(0.0) // external ks is used
                    .withKV(TURRET_FF_GAINS.kV)
                    .withKA(TURRET_FF_GAINS.kA)
            )
        }

        if (e1.isConnected && e2.isConnected) {
            motor.setPosition(getAbsoluteAngle())
            homed = true
        }
    }

    override fun updateInputs(input: TurretIO.TurretIOInputs) {
        if (e1.isConnected && e2.isConnected && (!homed || resetTurretPos.get())) {
            motor.setPosition(getAbsoluteAngle())
            homed = true
            resetTurretPos.set(false)
        }
        if (!openLoop) {
            motor.setControl(
                closedLoopControl
                    .withPosition(goal)
                    .withFeedForward(
                        Math.copySign(TURRET_FF_GAINS.kS, goal.minus(motor.position.value).baseUnitMagnitude())
                    )
            )
        }
        motorDisconnect.set(!motor.isConnected)
        e1Disconnect.set(!e1.isConnected)
        e2Disconnect.set(!e2.isConnected)

        input.angle = getAbsoluteAngle()
        input.motorAngle = motor.position.value
        input.speed = getVelocity()

        input.motorVoltageOut = motor.motorVoltage.value
        input.motorCurrentOut = motor.supplyCurrent.value
        input.motorTemp = motor.deviceTemp.value

        input.goal = goal
        input.atSetpoint = MathUtil.isNear(goal.`in`(Radians), motor.position.value.`in`(Radians), tolerance.`in`(Radians))

        input.setpointPos = Rotations.of(motor.closedLoopReference.value)
        input.setpointVel = RotationsPerSecond.of(motor.closedLoopReferenceSlope.value)

        input.openLoop = openLoop

        input.angleE1 = e1.absolutePosition.value
        input.angleE2 = e2.absolutePosition.value

        input.tolerance = tolerance
    }

    override fun setGoal(goal: Angle) {
        this.goal = goal
        openLoop = false
    }

    override fun setVout(vout: Voltage) {
        openLoop = true
        motor.setControl(openLoopControl.withOutput(vout))
    }

    private fun getAbsoluteAngle(): Angle {
        val angle: Angle
        if (e1.isConnected && e2.isConnected) {
            angle = Degrees.of(ExtraMath.calculateTurretAngleFromCANCoderDegrees(e1.absolutePosition.value.`in`(Degrees), e2.absolutePosition.value.`in`(Degrees)))
        } else {
            if (motor.isConnected && homed) {
                angle = motor.position.value
            } else {
                angle = Rotations.of(0.0)
            }
        }
        return angle
    }

    private fun getVelocity(): AngularVelocity {
        var measures = 0
        var sum = 0.0 // RPS

        if (e1.isConnected) {
            sum += e1.velocity.valueAsDouble / (TURRET_MAIN_GEAR_TEETH / TURRET_ENCODER_1_GEAR_TEETH)
            measures++
        }
        if (e2.isConnected) {
            sum += e2.velocity.valueAsDouble / (TURRET_MAIN_GEAR_TEETH / TURRET_ENCODER_2_GEAR_TEETH)
            measures++
        }
        if (motor.isConnected) {
            sum += motor.velocity.valueAsDouble
            measures++
        }
        if (measures > 0) {
            return RotationsPerSecond.of(-sum / measures) // average all velocity readings
        }
        return RotationsPerSecond.of(0.0) // the bad ending
    }

    override fun setTolerance(tol: Angle) {
        tolerance = tol
    }
}
