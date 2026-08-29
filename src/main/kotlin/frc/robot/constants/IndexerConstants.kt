package frc.robot.constants

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage

object IndexerConstants {
    const val INDEXER_MOTOR_ID: Int = 43

    const val INDEXER_MOTOR_INVERT: Boolean = false
    @JvmField val INDEXER_MAX_CURRENT: Current = Amps.of(40.0)

    @JvmField val INDEXER_FEED_VOLTAGE: Voltage = Volts.of(10.0)

    const val POSITION_CONVERSION_FACTOR: Double = 1.0
    @JvmField val VELOCITY_CONVERSION_FACTOR: Double = POSITION_CONVERSION_FACTOR * 60
}
