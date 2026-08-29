package frc.robot.subsystems.swerve.gyro

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusCode
import com.ctre.phoenix6.configs.Pigeon2Configuration
import com.ctre.phoenix6.hardware.Pigeon2
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Radian
import frc.robot.constants.DriveConstants.GYRO_ID
import frc.robot.constants.DriveConstants.ODOMETRY_FREQ
import frc.utils.PhoenixOdometryThread
import java.util.Queue

class GyroIOPigeon2 : GyroIO {
    private val pigeon = Pigeon2(GYRO_ID)
    private val yaw = pigeon.yaw
    private val yawPositionQueue: Queue<Double>
    private val yawTimestampQueue: Queue<Double>
    private val yawVelocity = pigeon.getAngularVelocityZWorld()

    init {
        pigeon.configurator.apply(Pigeon2Configuration())
        pigeon.configurator.setYaw(0.0)
        yaw.setUpdateFrequency(ODOMETRY_FREQ)
        yawVelocity.setUpdateFrequency(50.0)
        pigeon.getAccelerationX().setUpdateFrequency(50.0)
        pigeon.getAccelerationY().setUpdateFrequency(50.0)
        pigeon.getAccelerationZ().setUpdateFrequency(50.0)
        pigeon.optimizeBusUtilization()
        yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue()
        yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(yaw.clone())
    }

    override fun updateInputs(inputs: GyroIO.GyroIOInputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity) == StatusCode.OK
        inputs.yawPosition = yaw.value
        inputs.yawVelocity = yawVelocity.value

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble { it }.toArray()
        inputs.odometryYawPositions = yawPositionQueue.stream().mapToDouble { Units.degreesToRadians(it) }.toArray()
        yawTimestampQueue.clear()
        yawPositionQueue.clear()

        inputs.angle = Rotation3d(
            pigeon.roll.value.`in`(Radian),
            pigeon.pitch.value.`in`(Radian),
            pigeon.yaw.value.`in`(Radian)
        )

        inputs.accelX = pigeon.accelerationX.value
        inputs.accelY = pigeon.accelerationY.value
        inputs.accelZ = pigeon.accelerationZ.value
    }

    override fun reset(headingRad: Double) {
        pigeon.setYaw(Units.radiansToDegrees(headingRad))
    }
}
