package frc.robot.constants

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.Pounds
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.RobotBase

object Constants {

    enum class RobotMode {
        REAL,
        SIM,
        REPLAY
    }

    @JvmField var SIM_MODE: RobotMode = RobotMode.SIM
    @JvmField var MODE: RobotMode = if (RobotBase.isReal()) RobotMode.REAL else SIM_MODE

    @JvmField val AUTO_TIME: Time = Seconds.of(20.0)
    @JvmField val TELEOP_TIME: Time = Seconds.of(140.0)
    @JvmField val ENDGAME_TIME: Time = Seconds.of(30.0)
    const val EVENT_LOOP_TIME: Double = 0.02
    @JvmField val STARTING_POSE: Pose2d = Pose2d(Translation2d(8.75, 4.0), Rotation2d.fromDegrees(0.0))

    @JvmField val ROBOT_MASS: Mass = Pounds.of(115.0)

    const val ELASTIC_LAYOUT_PORT: Int = 3742

    object OperatorConstants {
        const val LEFT_DEADBAND: Double = 0.1
        const val RIGHT_DEADBAND: Double = 0.15

        const val TRANSLATION_CURVE: Double = 2.0
        const val ROTATION_CURVE: Double = 2.0

        const val DRIVER_CONTROLLER_PORT: Int = 0
        const val OPERATOR_CONTROLLER_PORT: Int = 1
    }

    object Drive {
        const val STARTING_FOD: Boolean = true
    }

    const val SPARKMAX_TARGET_FIRMWARE: Int = 436273157
    const val TALONFX_TARGET_FIRMWARE: Int = 436338688

    @JvmField val MAX_MOTOR_TEMP: Temperature = Celsius.of(80.0)
}
