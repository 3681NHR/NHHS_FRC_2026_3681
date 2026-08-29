package frc.robot.subsystems.hood

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.AutoLog

interface HoodIO {
    fun updateInputs(input: HoodIOInputs) {}

    fun setGoal(goal: Angle) {}
    fun setVout(vout: Voltage) {}
    fun setPos(pos: Angle) {}
    fun setHomed(homed: Boolean) {}
    fun reset() {}

    @AutoLog
    open class HoodIOInputs {
        @JvmField var angle: Angle = Radians.of(0.0)
        @JvmField var goal: Angle = Radians.of(0.0)
        @JvmField var setpointPos: Angle = Radians.of(0.0)
        @JvmField var velocity: AngularVelocity = RadiansPerSecond.of(0.0)
        @JvmField var homed: Boolean = false
        @JvmField var atSetpoint: Boolean = false

        @JvmField var vout: Voltage = Volts.of(0.0)
        @JvmField var current: Current = Amps.of(0.0)
        @JvmField var temp: Temperature = Celsius.of(0.0)

        @JvmField var openloop: Boolean = false
    }
}
