package frc.utils

import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj.Timer
import frc.robot.constants.Constants
import org.littletonrobotics.junction.Logger

/**
 * handles a timer for teleop and auto, sends all data to log/NT
 */
object TimerHandler {

    private var autoStart: Double = 0.0
    private var teleopStart: Double = 0.0

    @JvmStatic
    fun updateTeleop() {
        Logger.recordOutput("Utils/time/teleop/UpTime", Timer.getFPGATimestamp() - teleopStart)
        Logger.recordOutput(
            "Utils/time/teleop/RemainingTime",
            Math.max(0.0, Constants.TELEOP_TIME.`in`(Seconds) - (Timer.getFPGATimestamp() - teleopStart))
        )
        Logger.recordOutput(
            "Utils/time/RemainingTime",
            Math.max(0.0, Constants.TELEOP_TIME.`in`(Seconds) - (Timer.getFPGATimestamp() - teleopStart))
        )
    }

    @JvmStatic
    fun updateAuto() {
        Logger.recordOutput("Utils/time/auto/UpTime", Timer.getFPGATimestamp() - autoStart)
        Logger.recordOutput(
            "Utils/time/auto/RemainingTime",
            Math.max(0.0, Constants.AUTO_TIME.`in`(Seconds) - (Timer.getFPGATimestamp() - autoStart))
        )
        Logger.recordOutput(
            "Utils/time/RemainingTime",
            Math.max(0.0, Constants.AUTO_TIME.`in`(Seconds) - (Timer.getFPGATimestamp() - autoStart))
        )
    }

    @JvmStatic
    fun initTeleop() {
        teleopStart = Timer.getFPGATimestamp()
        Logger.recordOutput("Utils/time/teleop/StartTime", teleopStart)
    }

    @JvmStatic
    fun initAuto() {
        autoStart = Timer.getFPGATimestamp()
        Logger.recordOutput("Utils/time/auto/StartTime", autoStart)
    }

    @JvmStatic
    fun init() {
        Logger.recordOutput("Utils/time/upTime", 0.0)
        Logger.recordOutput("Utils/time/RemainingTime", 0.0)

        Logger.recordOutput("Utils/time/auto/StartTime", 0.0)
        Logger.recordOutput("Utils/time/auto/UpTime", 0.0)
        Logger.recordOutput("Utils/time/auto/RemainingTime", 0.0)
        Logger.recordOutput("Utils/time/auto/TotalTime", Constants.AUTO_TIME)

        Logger.recordOutput("Utils/time/teleop/StartTime", 0.0)
        Logger.recordOutput("Utils/time/teleop/UpTime", 0.0)
        Logger.recordOutput("Utils/time/teleop/RemainingTime", 0.0)
        Logger.recordOutput("Utils/time/teleop/TotalTime", Constants.TELEOP_TIME)
    }

    @JvmStatic
    fun update() {
        Logger.recordOutput("Utils/time/upTime", Timer.getFPGATimestamp())
    }

    @JvmStatic
    fun getTeleopRemaining(): Double {
        return Math.max(0.0, Constants.TELEOP_TIME.`in`(Seconds) - (Timer.getFPGATimestamp() - teleopStart))
    }

    @JvmStatic
    fun getAutoRemaining(): Double {
        return Math.max(0.0, Constants.AUTO_TIME.`in`(Seconds) - (Timer.getFPGATimestamp() - autoStart))
    }
}
