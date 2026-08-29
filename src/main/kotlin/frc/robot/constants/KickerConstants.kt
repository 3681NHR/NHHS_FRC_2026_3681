package frc.robot.constants

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage

object KickerConstants {
    const val KICKER_MOTOR_ID: Int = 41
    const val KICKER_SENSOR_ID: Int = 33

    const val KICKER_MOTOR_INVERT: Boolean = false
    @JvmField val KICKER_MAX_CURRENT: Current = Amps.of(40.0)

    @JvmField val KICKER_PRELOAD_STOP_DISTANCE: Distance = Inches.of(3.5)
    @JvmField val KICKER_UNLOAD_MAX_DISTANCE: Distance = Inches.of(3.5)

    @JvmField val KICKER_UNLOAD_VOLTAGE: Voltage = Volts.of(-11.0)
    @JvmField val KICKER_UNLOAD_PARTIAL_VOLTAGE: Voltage = Volts.zero()
    @JvmField val KICKER_FEED_VOLTAGE: Voltage = Volts.of(11.0)
}
