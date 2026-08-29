package frc.robot.subsystems.indexer

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Kelvin
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.AutoLog

interface IndexerIO {
    fun updateInputs(input: IndexerIOInputs) {}

    fun setVout(vout: Voltage) {}

    @AutoLog
    open class IndexerIOInputs {
        @JvmField var speed: AngularVelocity = RPM.zero()

        @JvmField var motorVoltageOut: Voltage = Volts.zero()
        @JvmField var motorCurrentOut: Current = Amps.zero()
        @JvmField var motorTemp: Temperature = Kelvin.zero().minus(Kelvin.one())

        @JvmField var motorConnected: Boolean = false
    }
}
