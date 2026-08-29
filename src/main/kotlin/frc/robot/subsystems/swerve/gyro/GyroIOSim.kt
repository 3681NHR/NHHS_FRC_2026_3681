package frc.robot.subsystems.swerve.gyro

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Units.Radians
import frc.utils.SparkUtil
import org.ironmaple.simulation.drivesims.GyroSimulation
import java.util.Arrays

open class GyroIOSim(private val gyro: GyroSimulation) : GyroIO {

    override fun reset(headingRad: Double) {
        gyro.setRotation(Rotation2d(headingRad))
    }

    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = true
        inputs.yawPosition = Radians.of(gyro.gyroReading.radians)
        inputs.yawVelocity = gyro.measuredAngularVelocity

        inputs.odometryYawPositions = Arrays.stream(gyro.cachedGyroReadings).mapToDouble { it.radians }.toArray()
        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps()

        inputs.angle = Rotation3d(0.0, 0.0, inputs.yawPosition.`in`(Radians))
    }
}
