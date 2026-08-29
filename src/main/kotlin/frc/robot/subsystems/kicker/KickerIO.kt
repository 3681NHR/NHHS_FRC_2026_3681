package frc.robot.subsystems.kicker

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Kelvin
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.AutoLog

interface KickerIO {
    fun updateInputs(input: KickerIOInputs) {}

    fun setVout(vout: Voltage) {}

    @AutoLog
    open class KickerIOInputs {
        @JvmField var speed: AngularVelocity = RPM.zero()

        @JvmField var motorVoltageOut: Voltage = Volts.zero()
        @JvmField var motorCurrentOut: Current = Amps.zero()
        @JvmField var motorTemp: Temperature = Kelvin.zero().minus(Kelvin.one())

        @JvmField var hasBall: Boolean = false
        @JvmField var distance: Distance = Meters.zero()

        @JvmField var motorConnected: Boolean = false
        @JvmField var sensorConnected: Boolean = false
    }
}
