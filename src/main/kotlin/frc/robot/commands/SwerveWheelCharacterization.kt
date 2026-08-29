package frc.robot.commands

import edu.wpi.first.math.kinematics.SwerveModuleState
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Milliseconds
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.constants.DriveConstants
import frc.robot.subsystems.swerve.Drive
import frc.utils.ExtraMath
import org.littletonrobotics.junction.Logger
import java.util.ArrayList

/**
 * auto routine to calculate swerve wheel radius
 */
class SwerveWheelCharacterization(
    private val drive: Drive
) : Command() {

    private val gyroreadings: ArrayList<AngularVelocity> = ArrayList()
    private val wheelreadings: ArrayList<Array<SwerveModuleState>> = ArrayList()

    private var startTimer: Time = Seconds.of(1.0)

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        drive.runAngleCharacterization(0.0)
    }

    override fun execute() {
        if (startTimer.`in`(Seconds) > 0) {
            startTimer = startTimer.minus(Milliseconds.of(20.0))
            if (startTimer.`in`(Seconds) < 0.5) {
                val out = 2.0
                drive.runAngleCharacterization(out)
            }
        } else {
            val out = 2.0
            drive.runAngleCharacterization(out)
            gyroreadings.add(drive.getAngulerVelocity())
            wheelreadings.add(drive.getModuleStates())
            val rad = drive.getAngulerVelocity().`in`(RadiansPerSecond) * DriveConstants.RADIUS.`in`(Meters) /
                ExtraMath.mean(
                    drive.getModuleStates()[0].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters),
                    drive.getModuleStates()[1].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters),
                    drive.getModuleStates()[2].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters),
                    drive.getModuleStates()[3].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters)
                )
            Logger.recordOutput("Wheel char/data/gyro radsPerSec", drive.getAngulerVelocity())
            Logger.recordOutput(
                "Wheel char/data/wheel radPerSec",
                2 * Math.PI * ExtraMath.mean(
                    drive.getModuleStates()[0].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters),
                    drive.getModuleStates()[1].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters),
                    drive.getModuleStates()[2].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters),
                    drive.getModuleStates()[3].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters)
                )
            )
            if (rad.isFinite()) {
                Logger.recordOutput("Wheel char/data/rad", Meters.of(rad).`in`(Inches))
            }
        }
        Logger.recordOutput("Wheel char/data/timer", startTimer)
    }

    override fun end(interrupted: Boolean) {
        drive.stop()
        val rads: ArrayList<Distance> = ArrayList()
        for (i in gyroreadings.indices) {
            rads.add(
                Meters.of(
                    gyroreadings[i].`in`(RadiansPerSecond) * DriveConstants.RADIUS.`in`(Meters) /
                        (2 * Math.PI * ExtraMath.mean(
                            wheelreadings[i][0].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters),
                            wheelreadings[i][1].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters),
                            wheelreadings[i][2].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters),
                            wheelreadings[i][3].speedMetersPerSecond / DriveConstants.Module.WHEEL_RAD.`in`(Meters)
                        ))
                )
            )
        }
        var sum = 0.0
        for (i in rads.indices) {
            sum += rads[i].`in`(Meters)
        }
        sum /= rads.size
        Logger.recordOutput("Wheel char/avg", Meters.of(sum))
        Logger.recordOutput("Wheel char/rads meters", rads.stream().mapToDouble { e -> e.`in`(Meters) }.toArray())
    }

    override fun isFinished(): Boolean {
        return DriverStation.isDisabled()
    }
}
