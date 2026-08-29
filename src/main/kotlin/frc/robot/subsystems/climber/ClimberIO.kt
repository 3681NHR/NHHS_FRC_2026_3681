package frc.robot.subsystems.climber

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.AutoLog

interface ClimberIO {
    fun updateInputs(input: ClimberIOInputs) {}

    fun setVoltage(voltage: Voltage) {}
    fun setGoal(goal: Distance) {}

    fun setPosition(goal: Distance) {}
    fun setHomed(homed: Boolean) {}

    @AutoLog
    open class ClimberIOInputs {
        @JvmField var position: Distance = Meters.zero()
        @JvmField var velocity: LinearVelocity = MetersPerSecond.zero()

        @JvmField var motorVoltageOut: Voltage = Volts.zero()
        @JvmField var motorCurrentOut: Current = Amps.zero()
        @JvmField var motorTemp: Temperature = Celsius.zero()

        @JvmField var goal: Distance = Meters.zero()

        @JvmField var velocitySetpoint: LinearVelocity = MetersPerSecond.zero()
        @JvmField var positionSetpoint: Distance = Meters.zero()
        @JvmField var atSetpoint: Boolean = false

        @JvmField var connected: Boolean = false
        @JvmField var openLoop: Boolean = false
        @JvmField var homed: Boolean = false
    }
}
