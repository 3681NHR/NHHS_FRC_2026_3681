package frc.robot.subsystems.swerve.gyro

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.Radian
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.LinearAcceleration
import org.littletonrobotics.junction.AutoLog

interface GyroIO {

    @AutoLog
    open class GyroIOInputs {
        @JvmField var connected: Boolean = false
        @JvmField var yawPosition: Angle = Radians.of(0.0)
        @JvmField var yawVelocity: AngularVelocity = RadiansPerSecond.of(0.0)
        @JvmField var odometryYawTimestamps: DoubleArray = doubleArrayOf()
        @JvmField var odometryYawPositions: DoubleArray = doubleArrayOf()
        @JvmField var angle: Rotation3d = Rotation3d()
        @JvmField var accelX: LinearAcceleration = MetersPerSecondPerSecond.zero()
        @JvmField var accelY: LinearAcceleration = MetersPerSecondPerSecond.zero()
        @JvmField var accelZ: LinearAcceleration = MetersPerSecondPerSecond.zero()
    }

    fun updateInputs(inputs: GyroIOInputs) {}

    fun reset(headingRad: Double) {}
}
