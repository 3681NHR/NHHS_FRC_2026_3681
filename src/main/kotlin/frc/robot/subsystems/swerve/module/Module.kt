package frc.robot.subsystems.swerve.module

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.kinematics.SwerveModulePosition
import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.robot.constants.DriveConstants
import org.littletonrobotics.junction.Logger

class Module(
    private val io: ModuleIO,
    private val index: Int
) {
    private val inputs = ModuleIOInputsAutoLogged()
    private val driveDisconnectedAlert = Alert(
        "Disconnected drive motor on module $index.",
        Alert.AlertType.kError
    )
    private val turnDisconnectedAlert = Alert(
        "Disconnected turn motor on module $index.", Alert.AlertType.kError
    )
    private var odometryPositions: Array<SwerveModulePosition> = emptyArray()

    fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("IO/Drive/Module$index", inputs)

        val sampleCount = inputs.odometryTimestamps.size
        odometryPositions = Array(sampleCount) { i ->
            val positionMeters = DriveConstants.Module.WHEEL_RAD.times(inputs.odometryDrivePositionsRad[i])
            val angle = Rotation2d(inputs.odometryTurnPositionsRad[i])
            SwerveModulePosition(positionMeters, angle)
        }

        driveDisconnectedAlert.set(!inputs.driveConnected)
        turnDisconnectedAlert.set(!inputs.turnConnected)
    }

    fun getOdometryPosition(i: Int): SwerveModulePosition {
        val positionMeters = DriveConstants.Module.WHEEL_RAD.times(inputs.odometryDrivePositionsRad[i])
        val angle = Rotation2d(inputs.odometryTurnPositionsRad[i])
        return SwerveModulePosition(positionMeters, angle)
    }

    fun runSetpoint(state: SwerveModuleState) {
        state.optimize(getAngle())
        val scaled = newCosineScale(state, Rotation2d(inputs.turnPosition))

        io.setDriveVelocity(RadiansPerSecond.of(scaled.speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters)))
        io.setTurnPosition(Radians.of(scaled.angle.radians))
    }

    fun runCharacterization(output: Voltage) {
        io.setDriveOpenLoop(output)
        io.setTurnPosition(Radians.of(0.0))
    }

    fun runCharacterization(output: Voltage, angle: Angle) {
        io.setDriveOpenLoop(output)
        io.setTurnPosition(angle)
    }

    fun runSteerCharacterization(output: Voltage) {
        io.setDriveOpenLoop(Volts.of(0.0))
        io.setTurnOpenLoop(output)
    }

    fun stop() {
        io.setDriveOpenLoop(Volts.of(0.0))
        io.setTurnOpenLoop(Volts.of(0.0))
    }

    fun idle() {
        io.setDriveVelocity(inputs.driveVelocity)
        io.setTurnPosition(inputs.turnPosition)
    }

    fun getAngle(): Rotation2d {
        return Rotation2d(inputs.turnPosition)
    }

    fun getDrivePosition(): Distance {
        return Meters.of(inputs.drivePosition.`in`(Radians) * DriveConstants.Module.WHEEL_RAD.`in`(Meters))
    }

    fun getDriveVelocity(): LinearVelocity {
        return MetersPerSecond.of(inputs.driveVelocity.`in`(RadiansPerSecond) * DriveConstants.Module.WHEEL_RAD.`in`(Meters))
    }

    fun getPosition(): SwerveModulePosition {
        return SwerveModulePosition(getDrivePosition(), getAngle())
    }

    fun getState(): SwerveModuleState {
        return SwerveModuleState(getDriveVelocity(), getAngle())
    }

    fun getOdometryTimestamps(): DoubleArray {
        return inputs.odometryTimestamps
    }

    companion object {
        @JvmStatic
        fun newCosineScale(state: SwerveModuleState, currentAngle: Rotation2d): SwerveModuleState {
            val angleDiff = state.angle.minus(currentAngle).radians
            val scale = Math.pow(Math.abs(Math.cos(angleDiff)) + 0.004, 2.0)
            val clamped = MathUtil.clamp(scale, -1.0, 1.0)
            val scaledSpeed = state.speedMetersPerSecond * clamped
            return SwerveModuleState(scaledSpeed, state.angle)
        }
    }
}
