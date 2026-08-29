package frc.robot.constants

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage
import frc.utils.controlWrappers.PIDGains

object ClimberConstants {
    const val CLIMBER_MOTOR_ID: Int = 5

    const val CLIMBER_INVERTED: Boolean = false
    @JvmField val CLIMBER_CURRENT_LIM: Current = Amps.of(30.0)

    @JvmField val CLIMBER_MAX_POSITION: Distance = Inches.of(16.0)
    @JvmField val CLIMBER_MIN_POSITION: Distance = Meters.of(0.0)

    @JvmField val CLIMBER_SETPOINT_TOLERANCE: Distance = Inches.of(1.0)

    @JvmField val CLIMBER_ID_GAINS: PIDGains.GravityFF = PIDGains.GravityFF(0.1, 0.0, 0.1, 0.0001)

    @JvmField val CLIMBER_FF_GAINS: PIDGains.GravityFF = PIDGains.GravityFF(0.1, 0.0, 0.1, 0.0).makeTunable("Tuning/Climber/FF")
    @JvmField val CLIMBER_PID_GAINS: PIDGains.ProfiledPID = PIDGains.ProfiledPID(0.1, 0.0, 0.0, 10.0, 100.0).makeTunable("Tuning/Climber/PID")

    private val CLIMBER_SPOOL_RADIUS: Distance = Inches.of(1.0)
    private const val CLIMBER_GEAR_RATIO: Double = 80.0
    @JvmField val CLIMBER_POSITION_CONVERSION_FACTOR: Double = (CLIMBER_SPOOL_RADIUS.`in`(Meters) * 2 * Math.PI) / CLIMBER_GEAR_RATIO
    @JvmField val CLIMBER_VELOCITY_CONVERSION_FACTOR: Double = CLIMBER_POSITION_CONVERSION_FACTOR

    const val CLIMBER_HOME_ON_START: Boolean = false
    @JvmField val CLIMBER_HOME_VOLTAGE: Voltage = Volts.of(-2.0)
    @JvmField val CLIMBER_HOME_STOP_TIME: Time = Seconds.of(0.5)
    @JvmField val CLIMBER_HOME_STOP_THRESH: LinearVelocity = MetersPerSecond.of(0.01)
}
