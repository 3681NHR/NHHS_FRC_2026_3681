package frc.robot.subsystems.turret

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.Rotations
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.constants.TurretConstants.TURRET_ANGLE_FORWARD_LIM
import frc.robot.constants.TurretConstants.TURRET_ANGLE_REVERSE_LIM
import frc.robot.constants.TurretConstants.TURRET_OFFSET
import frc.robot.constants.TurretConstants.TURRET_SETPOINT_TOLERANCE
import frc.robot.constants.TurretConstants.TURRET_SYSID_CONFIG
import frc.robot.constants.TurretConstants.TURRET_THETA_COMP_FACTOR
import frc.robot.subsystems.SOTMSolver
import frc.robot.subsystems.swerve.Drive
import frc.utils.ExtraMath
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class Turret(
    private val io: TurretIO,
    private val drive: Drive
) : SubsystemBase() {

    private var ready = false

    private val inputs = TurretIOInputsAutoLogged()

    private val sysid = SysIdRoutine(TURRET_SYSID_CONFIG, SysIdRoutine.Mechanism({ v -> io.setVout(v) }, null, this))

    private val illegalTarg = Alert("illegal or invalid Turret setpoint!", Alert.AlertType.kWarning)

    private val runningSysid = Alert("Turret sysid running", Alert.AlertType.kInfo)

    init {
        // init log values
        Logger.recordOutput("Subsystems/Turret/track/angle targeted", Double.NaN)
        Logger.recordOutput("Subsystems/Turret/track/init angle targeted", Double.NaN)
        Logger.recordOutput("Subsystems/Turret/track/target pos", null as Translation2d?)

        Logger.recordOutput("Subsystems/Turret/manual/target", Double.NaN)
    }

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("IO/Turret", inputs)

        Logger.recordOutput("Subsystems/Turret/state", currentCommand?.name ?: "none")

        Logger.recordOutput(
            "Subsystems/Turret/field angle",
            inputs.motorAngle
                .plus(Radians.of(drive.rotation.radians))
                .plus(if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) Degrees.of(180.0) else Degrees.of(0.0)).`in`(Rotations),
            Rotations
        )
    }

    fun manPos(targ: Supplier<Angle>, fieldOriented: Boolean): Command {
        return Commands.run(Runnable {
            var angle = targ.get()

            io.setTolerance(TURRET_SETPOINT_TOLERANCE)

            if (fieldOriented) {
                angle = angle
                    .minus(Radians.of(drive.pose.rotation.radians))
                    .plus(Radians.of(TURRET_THETA_COMP_FACTOR * drive.angulerVelocity.`in`(RadiansPerSecond)))
            }

            if (angle.gt(TURRET_ANGLE_FORWARD_LIM) || angle.lt(TURRET_ANGLE_REVERSE_LIM)) {
                ready = false
                illegalTarg.set(true)
            } else {
                io.setGoal(angle)
                ready = inputs.atSetpoint
                illegalTarg.set(false)
            }

            Logger.recordOutput("Subsystems/Turret/manual/target", angle)
        }, this).finallyDo(Runnable {
            Logger.recordOutput("Subsystems/Turret/manual/target", Double.NaN)
        }).withName("manual angle")
    }

    fun track(targ: Supplier<Translation2d>, radius: Supplier<Distance>): Command {
        return Commands.run(Runnable {
            val dist = targ.get().getDistance(getFieldPos())
            Logger.recordOutput("Subsystems/Turret/track/distance", dist)

            io.setTolerance(Radians.of(2 * Math.atan(radius.get().`in`(Meters) / dist)))

            var angle = ExtraMath.getAngleToPos(
                targ.get(), // target(offset for lead)
                drive.pose.translation // drive pos
                    .plus(
                        Translation2d( // turret offset
                            Math.cos(drive.rotation.radians) * TURRET_OFFSET.x,
                            Math.sin(drive.rotation.radians) * TURRET_OFFSET.x
                        )
                    )
            )
                .minus(Radians.of(drive.pose.rotation.radians))
                .plus(Radians.of(TURRET_THETA_COMP_FACTOR * drive.angulerVelocity.`in`(RadiansPerSecond)))

            val finalAngle = Degrees.of(
                convertToClosestBoundedTurretAngleDegrees(
                    angle.`in`(Degrees),
                    Rotation2d(inputs.motorAngle.`in`(Radians)),
                    TURRET_ANGLE_FORWARD_LIM.`in`(Degrees),
                    TURRET_ANGLE_REVERSE_LIM.`in`(Degrees)
                )
            )
            io.setGoal(finalAngle)

            ready = inputs.atSetpoint

            Logger.recordOutput("Subsystems/Turret/track/angle targeted", finalAngle)
            Logger.recordOutput("Subsystems/Turret/track/init angle targeted", angle)
            Logger.recordOutput("Subsystems/Turret/track/target pos", targ.get())
        }, this).finallyDo(Runnable {
            Logger.recordOutput("Subsystems/Turret/track/angle targeted", Double.NaN)
            Logger.recordOutput("Subsystems/Turret/track/init angle targeted", Double.NaN)
            Logger.recordOutput("Subsystems/Turret/track/target pos", null as Translation2d?)
            io.setTolerance(TURRET_SETPOINT_TOLERANCE)
        }).withName("track position")
    }

    fun trackWithLead(radius: Supplier<Distance>): Command {
        return Commands.run(Runnable {
            io.setTolerance(Radians.of(2 * Math.atan(radius.get().`in`(Meters) / SOTMSolver.getInstance().getParams(false).dist().`in`(Meters))))

            var angle = SOTMSolver.getInstance().getAngle(false)
                .minus(Radians.of(drive.pose.rotation.radians))
                .plus(Radians.of(TURRET_THETA_COMP_FACTOR * drive.angulerVelocity.`in`(RadiansPerSecond)))

            val finalAngle = Degrees.of(
                convertToClosestBoundedTurretAngleDegrees(
                    angle.`in`(Degrees),
                    Rotation2d(inputs.motorAngle.`in`(Radians)),
                    TURRET_ANGLE_FORWARD_LIM.`in`(Degrees),
                    TURRET_ANGLE_REVERSE_LIM.`in`(Degrees)
                )
            )
            io.setGoal(finalAngle)

            ready = inputs.atSetpoint

            Logger.recordOutput("Subsystems/Turret/track/angle targeted", finalAngle)
            Logger.recordOutput("Subsystems/Turret/track/init angle targeted", angle)
            Logger.recordOutput("Subsystems/Turret/track/target pos", SOTMSolver.getInstance().target)
        }, this).finallyDo(Runnable {
            Logger.recordOutput("Subsystems/Turret/track/angle targeted", Double.NaN)
            Logger.recordOutput("Subsystems/Turret/track/init angle targeted", Double.NaN)
            Logger.recordOutput("Subsystems/Turret/track/target pos", null as Translation2d?)
            io.setTolerance(TURRET_SETPOINT_TOLERANCE)
        }).withName("track position from SOTM")
    }

    fun sysidQuasistatic(reverse: Boolean): Command {
        return sysid.quasistatic(if (reverse) SysIdRoutine.Direction.kReverse else SysIdRoutine.Direction.kForward)
            .raceWith(Commands.run(Runnable {
                ready = false
                runningSysid.set(true)
                runningSysid.setText("Turret sysid running: Quasistatic, " + if (reverse) "reverse" else "forward")
            }))
            .finallyDo(Runnable {
                runningSysid.set(false)
            })
            .withName("Quasistatic sysid: " + if (reverse) "reverse" else "forward")
    }

    fun sysidDynamic(reverse: Boolean): Command {
        return sysid.dynamic(if (reverse) SysIdRoutine.Direction.kReverse else SysIdRoutine.Direction.kForward)
            .raceWith(Commands.run(Runnable {
                ready = false
                runningSysid.set(true)
                runningSysid.setText("Turret sysid running: Dynamic, " + if (reverse) "reverse" else "forward")
            }))
            .finallyDo(Runnable {
                runningSysid.set(false)
            })
            .withName("Dynamic sysid: " + if (reverse) "reverse" else "forward")
    }

    @AutoLogOutput(key = "Subsystems/Turret/ready")
    fun isReady(): Boolean {
        return ready
    }

    fun getAngle(): Angle {
        return inputs.motorAngle
    }

    fun getAbsoluteAngle(): Angle {
        return inputs.angle
    }

    fun getFieldPos(): Translation2d {
        return drive.pose.translation // drive pos
            .plus(
                Translation2d( // turret offset
                    Math.cos(drive.rotation.radians) * TURRET_OFFSET.x,
                    Math.sin(drive.rotation.radians) * TURRET_OFFSET.x
                )
            )
    }

    fun stop(): Command {
        return InstantCommand(Runnable { Volts.of(0.0) })
    }

    companion object {
        /**
         * Sets the robot-relative target angle for the turret.
         * First the closest path from current turret angle to the target angle is calculated.
         * If the path is found to be move outside the bounds, the path will adjust to follow the next closest path.
         *
         * @param targetAngleDegrees Target angle in degrees
         * @param current Current turret angle
         *
         * @return next absolute angle in degrees for the robot to move to
         *
         * (thanks 2910)
         */
        @JvmStatic
        fun convertToClosestBoundedTurretAngleDegrees(
            targetAngleDegrees: Double,
            current: Rotation2d,
            forwardLimitDegrees: Double,
            reverseLimitDegrees: Double
        ): Double {
            val currentTotalRadians = (current.rotations * 2 * Math.PI)
            var closestOffset = Units.degreesToRadians(targetAngleDegrees) - current.radians
            if (closestOffset > Math.PI) {
                closestOffset -= 2 * Math.PI
            } else if (closestOffset < -Math.PI) {
                closestOffset += 2 * Math.PI
            }

            var finalOffset = currentTotalRadians + closestOffset
            if ((currentTotalRadians + closestOffset) % (2 * Math.PI) == (currentTotalRadians - closestOffset) % (2 * Math.PI)) { // If the offset can go either way, go closer to zero
                if (finalOffset > 0) {
                    finalOffset = currentTotalRadians - Math.abs(closestOffset)
                } else {
                    finalOffset = currentTotalRadians + Math.abs(closestOffset)
                }
            }
            if (finalOffset > Units.degreesToRadians(forwardLimitDegrees)) { // if past upper rotation limit
                finalOffset -= (2 * Math.PI)
            } else if (finalOffset < Units.degreesToRadians(reverseLimitDegrees)) { // if below lower rotation limit
                finalOffset += (2 * Math.PI)
            }

            return Units.radiansToDegrees(finalOffset)
        }
    }
}
