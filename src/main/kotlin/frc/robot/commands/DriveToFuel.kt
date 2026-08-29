package frc.robot.commands

import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import frc.robot.constants.FuelVisionConstants
import frc.robot.subsystems.fuelVision.FuelVision
import frc.robot.subsystems.swerve.Drive
import frc.utils.ExtraMath
import frc.utils.RectZone
import org.littletonrobotics.junction.Logger
import java.util.Comparator
import java.util.function.DoubleSupplier
import java.util.function.Supplier

class DriveToFuel(
    private val drive: Drive,
    private val fuelVision: FuelVision,
    private val zone: Supplier<RectZone>
) : Command() {

    private val driveCommand: Command

    private var angleToTarg: Double = 0.0
    private var fieldVX: Double = 0.0
    private var fieldVY: Double = 0.0

    init {
        driveCommand = drive.rotationLock(
            DoubleSupplier { angleToTarg },
            DoubleSupplier { fieldVX },
            DoubleSupplier { fieldVY }
        )
        CommandScheduler.getInstance().registerComposedCommands(driveCommand)

        addRequirements(drive, fuelVision)
    }

    override fun initialize() {
        driveCommand.initialize()
    }

    override fun execute() {
        val map = fuelVision.getFuelMap()

        val best = map.entries.stream()
            .filter { entry ->
                zone.get().contains(
                    Translation2d(
                        entry.key.x() * FuelVisionConstants.GRID_SIZE.`in`(Meters),
                        entry.key.y() * FuelVisionConstants.GRID_SIZE.`in`(Meters)
                    )
                )
            }
            .max(
                Comparator.comparingDouble { entry ->
                    val fuel = entry.value.size.toDouble()
                    val dist = Translation2d(
                        entry.key.x() * FuelVisionConstants.GRID_SIZE.`in`(Meters),
                        entry.key.y() * FuelVisionConstants.GRID_SIZE.`in`(Meters)
                    ).getDistance(drive.pose.translation)
                    fuel / Math.max(dist, 0.01)
                }
            )

        if (best.isPresent && best.get().value.isNotEmpty()) {
            val target = Translation2d(
                best.get().key.x() * FuelVisionConstants.GRID_SIZE.`in`(Meters),
                best.get().key.y() * FuelVisionConstants.GRID_SIZE.`in`(Meters)
            )
            angleToTarg = ExtraMath.getAngleToPos(target, drive.pose.translation).`in`(Radians)

            fieldVX = Math.cos(drive.rotation.radians - angleToTarg) * Math.cos(angleToTarg)
            fieldVY = Math.cos(drive.rotation.radians - angleToTarg) * Math.sin(angleToTarg)

            Logger.recordOutput("Auto/AI/target", target)
        } else {
            Logger.recordOutput("Auto/AI/target", null as Translation2d?)
            angleToTarg = drive.rotation.plus(Rotation2d.fromDegrees(180 * 0.02)).radians
            fieldVX = 0.0
            fieldVY = 0.0
        }
        driveCommand.execute()
    }

    override fun end(interrupted: Boolean) {
        driveCommand.end(interrupted)
    }

    override fun isFinished(): Boolean {
        return driveCommand.isFinished
    }
}
