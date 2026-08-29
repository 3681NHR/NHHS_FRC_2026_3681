package frc.robot.subsystems.climber

import com.revrobotics.PersistMode
import com.revrobotics.REVLibError
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.robot.constants.ClimberConstants.CLIMBER_FF_GAINS
import frc.robot.constants.ClimberConstants.CLIMBER_HOME_ON_START
import frc.robot.constants.ClimberConstants.CLIMBER_INVERTED
import frc.robot.constants.ClimberConstants.CLIMBER_MIN_POSITION
import frc.robot.constants.ClimberConstants.CLIMBER_MOTOR_ID
import frc.robot.constants.ClimberConstants.CLIMBER_PID_GAINS
import frc.robot.constants.ClimberConstants.CLIMBER_POSITION_CONVERSION_FACTOR
import frc.robot.constants.ClimberConstants.CLIMBER_SETPOINT_TOLERANCE
import frc.robot.constants.ClimberConstants.CLIMBER_VELOCITY_CONVERSION_FACTOR
import frc.utils.controlWrappers.ElevatorFF
import frc.utils.controlWrappers.ProfiledPID
import frc.utils.motorWrappers.SparkMax

class ClimberIOReal : ClimberIO {

    private val motor = SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless)
    private val encoder = motor.encoder

    private val pid = ProfiledPID(CLIMBER_PID_GAINS)
    private val ff = ElevatorFF(CLIMBER_FF_GAINS)

    private var openLoop = false
    private val disconnect = Alert("Climber Spark is disconnected!", Alert.AlertType.kError)

    private var goal: Distance = CLIMBER_MIN_POSITION

    private var homed: Boolean = CLIMBER_HOME_ON_START

    init {
        CLIMBER_PID_GAINS.withCallback {
            pid.setGains(CLIMBER_PID_GAINS)
        }
        CLIMBER_FF_GAINS.withCallback {
            ff.setKs(CLIMBER_FF_GAINS.kS)
            ff.setKv(CLIMBER_FF_GAINS.kV)
            ff.setKa(CLIMBER_FF_GAINS.kA)
            ff.setKg(CLIMBER_FF_GAINS.kG)
        }

        val motorConfig = SparkMaxConfig()

        motorConfig.idleMode(IdleMode.kBrake).inverted(CLIMBER_INVERTED)
        motorConfig.encoder
            .positionConversionFactor(CLIMBER_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(CLIMBER_VELOCITY_CONVERSION_FACTOR)

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        pid.setTolerance(CLIMBER_SETPOINT_TOLERANCE.`in`(Meters))
    }

    override fun updateInputs(input: ClimberIO.ClimberIOInputs) {
        if (!openLoop && homed) {
            motor.setVoltage(Units.Volts.of(pid.calculate(encoder.position, goal.`in`(Meters)) + ff.calculate(pid.setpoint.velocity)))
        }
        input.position = Units.Meters.of(encoder.position)
        input.velocity = Units.MetersPerSecond.of(encoder.velocity)

        input.motorVoltageOut = Units.Volts.of(motor.appliedOutput * motor.busVoltage)
        input.motorCurrentOut = Units.Amps.of(motor.outputCurrent)
        input.motorTemp = Units.Celsius.of(motor.motorTemperature)

        input.goal = goal
        input.positionSetpoint = Units.Meters.of(pid.setpoint.position)
        input.velocitySetpoint = Units.MetersPerSecond.of(pid.setpoint.velocity)
        input.atSetpoint = pid.atSetpoint()

        input.connected = motor.getLastError() != REVLibError.kCANDisconnected
        input.openLoop = openLoop
        input.homed = homed

        disconnect.set(!input.connected)
    }

    override fun setVoltage(voltage: Voltage) {
        openLoop = true
        motor.setVoltage(voltage)
    }

    override fun setGoal(goal: Distance) {
        openLoop = false
        this.goal = goal
    }

    override fun setHomed(homed: Boolean) {
        this.homed = homed
    }

    override fun setPosition(position: Distance) {
        encoder.setPosition(position.`in`(Meters))
    }
}
