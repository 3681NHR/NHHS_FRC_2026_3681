package frc.robot.constants

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage
import frc.utils.controlWrappers.PIDGains

object HoodConstants {
    const val HOOD_MOTOR_ID: Int = 51

    @JvmField val HOOD_SETPOINT_TOLERANCE: Angle = Degrees.of(3.0)

    @JvmField val HOOD_MIN_ANGLE: Angle = Degrees.of(25.0)
    @JvmField val HOOD_MAX_ANGLE: Angle = Degrees.of(47.0)

    @JvmField val HOOD_ID_GAINS: PIDGains.SimpleFF = PIDGains.SimpleFF(0.3, 35.0, 0.0000001)

    @JvmField val HOOD_PID_GAINS: PIDGains.ProfiledPID = PIDGains.ProfiledPID(40.0, 0.0, 5.0, 0.2, 3.0).makeTunable("Tuning/Hood/PID")
    @JvmField val HOOD_FF_GAINS: PIDGains.SimpleFF = PIDGains.SimpleFF(0.75, 25.0, 0.0).makeTunable("Tuning/Hood/FF")

    const val HOOD_HOME_ON_START: Boolean = false
    @JvmField val HOOD_HOME_VOLTAGE: Voltage = Volts.of(-2.0)
    @JvmField val HOOD_HOME_STOP_TIME: Time = Seconds.of(0.5)
    @JvmField val HOOD_HOME_STOP_THRESH: AngularVelocity = RadiansPerSecond.of(0.005)

    @JvmField val HOOD_GEAR_RATIO: Double = (1.0 / 25.0) * (10.0 / 176.0) * 0.750671851307

    @JvmField val HOOD_CURRENT_LIM: Current = Amps.of(20.0)
}
