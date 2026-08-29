package frc.robot.subsystems.hood

import com.revrobotics.PersistMode
import com.revrobotics.ResetMode
import com.revrobotics.spark.FeedbackSensor
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import frc.robot.constants.HoodConstants.HOOD_CURRENT_LIM
import frc.robot.constants.HoodConstants.HOOD_FF_GAINS
import frc.robot.constants.HoodConstants.HOOD_GEAR_RATIO
import frc.robot.constants.HoodConstants.HOOD_HOME_ON_START
import frc.robot.constants.HoodConstants.HOOD_MAX_ANGLE
import frc.robot.constants.HoodConstants.HOOD_MIN_ANGLE
import frc.robot.constants.HoodConstants.HOOD_MOTOR_ID
import frc.robot.constants.HoodConstants.HOOD_PID_GAINS
import frc.robot.constants.HoodConstants.HOOD_SETPOINT_TOLERANCE
import frc.utils.controlWrappers.ProfiledPID
import frc.utils.controlWrappers.SimpleFF
import frc.utils.motorWrappers.SparkMax

class HoodIOReal : HoodIO {

    private val motor = SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless)
    private val encoder = motor.getEncoder()

    private var homed: Boolean = HOOD_HOME_ON_START
    private var openloop: Boolean = false

    private var goal: Angle = Radians.of(0.0)
    private var vout: Voltage = Volts.of(0.0)

    private val pid = ProfiledPID(HOOD_PID_GAINS)
    private val ff = SimpleFF(HOOD_FF_GAINS)

    init {
        val turnConfig = SparkMaxConfig()
        turnConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(HOOD_CURRENT_LIM.`in`(Amps).toInt())
            .voltageCompensation(12.0)
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        turnConfig.encoder
            .positionConversionFactor(HOOD_GEAR_RATIO)
            .velocityConversionFactor(HOOD_GEAR_RATIO)
        motor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        HOOD_PID_GAINS.withCallback {
            pid.setGains(HOOD_PID_GAINS)
        }
        HOOD_FF_GAINS.withCallback {
            ff.setKs(HOOD_FF_GAINS.kS)
            ff.setKv(HOOD_FF_GAINS.kV)
            ff.setKa(HOOD_FF_GAINS.kA)
        }

        pid.setTolerance(HOOD_SETPOINT_TOLERANCE.`in`(Rotations))
    }

    override fun updateInputs(input: HoodIO.HoodIOInputs) {
        if (!openloop && homed) {
            vout = Volts.of(pid.calculate(encoder.position, goal.`in`(Rotations)))
            vout = vout.plus(Volts.of(ff.calculate(pid.setpoint.velocity)))
        }
        if (homed) {
            // soft limit - cant use internal, as it cant be configured while enabled
            if (encoder.position >= HOOD_MAX_ANGLE.`in`(Rotations) && vout.`in`(Volts) > 0) {
                vout = Volts.of(0.0)
            }
            if (encoder.position <= HOOD_MIN_ANGLE.`in`(Rotations) && vout.`in`(Volts) < 0) {
                vout = Volts.of(0.0)
            }
        }
        motor.setVoltage(vout.`in`(Volts))

        input.angle = Rotations.of(encoder.position)
        input.velocity = RPM.of(encoder.velocity)

        input.atSetpoint = pid.atGoal()

        input.vout = Volts.of(motor.appliedOutput * motor.busVoltage)
        input.current = Amps.of(motor.outputCurrent)
        input.temp = Celsius.of(motor.motorTemperature)

        input.homed = homed
        input.openloop = openloop

        input.goal = goal
        input.setpointPos = Rotations.of(pid.setpoint.position)
    }

    override fun setGoal(goal: Angle) {
        openloop = false
        this.goal = goal
    }

    override fun setVout(vout: Voltage) {
        openloop = true
        this.vout = vout
    }

    override fun setPos(pos: Angle) {
        encoder.setPosition(pos.`in`(Rotations))
    }

    override fun setHomed(homed: Boolean) {
        this.homed = homed
    }

    override fun reset() {
        pid.reset(encoder.position)
    }
}
