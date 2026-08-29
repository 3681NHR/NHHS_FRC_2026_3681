package frc.robot.subsystems.turret

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Kelvin
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.AutoLog

interface TurretIO {
    fun updateInputs(input: TurretIOInputs) {}

    fun setGoal(goal: Angle) {}
    fun setVout(vout: Voltage) {}

    fun setTolerance(tol: Angle) {}

    @AutoLog
    open class TurretIOInputs {
        @JvmField var angle: Angle = Radians.of(0.0)
        @JvmField var motorAngle: Angle = Radians.of(0.0)
        @JvmField var speed: AngularVelocity = RadiansPerSecond.of(0.0)
        @JvmField var tolerance: Angle = Radians.of(0.0)

        @JvmField var motorVoltageOut: Voltage = Volts.of(0.0)
        @JvmField var motorCurrentOut: Current = Amps.of(0.0)
        @JvmField var motorTemp: Temperature = Kelvin.of(0.0)

        @JvmField var goal: Angle = Radians.of(0.0)
        @JvmField var setpointPos: Angle = Radians.of(0.0)
        @JvmField var setpointVel: AngularVelocity = RadiansPerSecond.of(0.0)
        @JvmField var atSetpoint: Boolean = false

        @JvmField var openLoop: Boolean = false

        @JvmField var angleE1: Angle = Radians.of(0.0)
        @JvmField var angleE2: Angle = Radians.of(0.0)
    }
}
