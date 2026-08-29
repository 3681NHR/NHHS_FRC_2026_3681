package frc.robot.subsystems.swerve.module

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

interface ModuleIO {

    @AutoLog
    open class ModuleIOInputs {
        @JvmField var driveConnected: Boolean = false
        @JvmField var driveOpenLoop: Boolean = false
        @JvmField var drivePosition: Angle = Radians.of(0.0)
        @JvmField var driveVelocity: AngularVelocity = RadiansPerSecond.of(0.0)
        @JvmField var driveGoal: AngularVelocity = RadiansPerSecond.of(0.0)
        @JvmField var driveSetpoint: AngularVelocity = RadiansPerSecond.of(0.0)
        @JvmField var driveAppliedVolts: Voltage = Volts.of(0.0)
        @JvmField var driveCurrent: Current = Amps.of(0.0)
        @JvmField var driveTemp: Temperature = Celsius.of(0.0)

        @JvmField var turnConnected: Boolean = false
        @JvmField var turnOpenLoop: Boolean = false
        @JvmField var turnPosition: Angle = Radians.of(0.0)
        @JvmField var turnGoal: Angle = Radians.of(0.0)
        @JvmField var turnSetpoint: Angle = Radians.of(0.0)
        @JvmField var turnVelocity: AngularVelocity = RadiansPerSecond.of(0.0)
        @JvmField var turnAppliedVolts: Voltage = Volts.of(0.0)
        @JvmField var turnCurrent: Current = Amps.of(0.0)
        @JvmField var turnTemp: Temperature = Celsius.of(0.0)

        @JvmField var odometryTimestamps: DoubleArray = doubleArrayOf()
        @JvmField var odometryDrivePositionsRad: DoubleArray = doubleArrayOf()
        @JvmField var odometryTurnPositionsRad: DoubleArray = doubleArrayOf()
    }

    fun updateInputs(inputs: ModuleIOInputs) {}

    fun setDriveOpenLoop(output: Voltage) {}

    fun setTurnOpenLoop(output: Voltage) {}

    fun setDriveVelocity(velocity: AngularVelocity) {}

    fun setTurnPosition(rotation: Angle) {}
}
