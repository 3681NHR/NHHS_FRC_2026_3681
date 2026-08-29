package frc.robot.subsystems.intake

import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Voltage
import org.littletonrobotics.junction.AutoLog

interface IntakeIO {
    fun updateInputs(input: IntakeIOInputs) {}

    fun setRollerVoltage(voltage: Voltage) {}

    fun setPivotGoal(goal: Angle) {}
    fun setPivotVoltage(voltage: Voltage) {}

    @AutoLog
    open class IntakeIOInputs {
        // Roller
        @JvmField var rollerVelocity: AngularVelocity = RPM.zero()

        @JvmField var rollerVoltageOut: Voltage = Volts.zero()
        @JvmField var rollerCurrentOut: Current = Amps.zero()
        @JvmField var rollerTemp: Temperature = Celsius.zero()

        @JvmField var rollerConnected: Boolean = false

        // Pivot
        @JvmField var pivotAngle: Angle = Radians.zero()
        @JvmField var pivotVelocity: AngularVelocity = RadiansPerSecond.zero()

        @JvmField var pivotVoltageOut: Voltage = Volts.zero()
        @JvmField var pivotCurrentOut: Current = Amps.zero()
        @JvmField var pivotTemp: Temperature = Celsius.zero()

        @JvmField var pivotGoal: Angle = Radians.zero()
        @JvmField var pivotSetpointPos: Angle = Radians.zero()
        @JvmField var pivotSetpointVel: AngularVelocity = RadiansPerSecond.zero()
        @JvmField var pivotAtSetpoint: Boolean = false

        @JvmField var pivotOpenLoop: Boolean = false
        @JvmField var pivotMotorConnected: Boolean = false
        @JvmField var pivotEncoderConnected: Boolean = false
    }
}
