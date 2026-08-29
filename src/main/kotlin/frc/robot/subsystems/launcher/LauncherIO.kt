package frc.robot.subsystems.launcher

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.AutoLog

interface LauncherIO {
    fun updateInputs(input: LauncherIOInputs) {}

    fun setGoal(goal: AngularVelocity) {}
    fun setVout(vout: Voltage) {}

    @AutoLog
    open class LauncherIOInputs {
        @JvmField var angle: Angle = Rotations.zero()
        @JvmField var speed: AngularVelocity = RPM.zero()

        @JvmField var motorVoltageOut: Voltage = Volts.zero()
        @JvmField var motorCurrentOut: Current = Amps.zero()
        @JvmField var motorTemp: Temperature = Celsius.zero()

        @JvmField var goal: AngularVelocity = RPM.zero()
        @JvmField var atSetpoint: Boolean = false

        @JvmField var openLoop: Boolean = false
    }
}
