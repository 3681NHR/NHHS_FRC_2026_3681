package frc.robot.subsystems.fuelVision

import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.Pair
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Microseconds
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.FuelVisionConstants.CAMERA_CLEAR_HFOV
import frc.robot.constants.FuelVisionConstants.CAMERA_CONFIG
import frc.robot.constants.FuelVisionConstants.FUEL_OVERLAP_THRESH
import frc.robot.constants.FuelVisionConstants.FUEL_PERSISTANCE_TIME
import frc.robot.constants.FuelVisionConstants.FUEL_POV_CLEAR_GRACE_TIME
import frc.robot.constants.FuelVisionConstants.FUEL_RADIUS
import frc.robot.constants.FuelVisionConstants.GRID_SIZE
import frc.robot.constants.FuelVisionConstants.MAX_CLEAR_DETECTION_DIST
import frc.robot.constants.FuelVisionConstants.MAX_DETECTION_DIST
import frc.robot.constants.FuelVisionConstants.MIN_CLEAR_DETECTION_DIST
import frc.robot.constants.FuelVisionConstants.MIN_DETECTION_DIST
import frc.robot.constants.FuelVisionConstants.ROBOT_WIDTH
import frc.utils.ExtraMath
import java.util.ArrayList
import java.util.HashMap
import java.util.HashSet
import java.util.function.Supplier
import java.util.stream.Collectors
import org.littletonrobotics.junction.Logger

class FuelVision(
    private val io: FuelVisionIO,
    private val pose: Supplier<Pose2d>
) : SubsystemBase() {
    private val inputs = FuelVisionIOInputsAutoLogged()
    private val newFuel: ArrayList<fuelData> = ArrayList()
    private val fuelMap: MutableMap<gridCoord, MutableSet<fuelData>> = HashMap()
    private var driveHistory: MutableList<Pair<Long, Pose2d>> = ArrayList(10)
    var cellsToRender: ArrayList<Translation3d> = ArrayList()

    override fun periodic() {
        inputs.timestamp = Double.NaN
        io.updateInputs(inputs)
        Logger.processInputs("IO/FuelVision", inputs)

        val currTimestamp = Logger.getTimestamp()

        while (driveHistory.size > 10) {
            driveHistory.removeAt(0)
        }

        driveHistory.add(Pair(currTimestamp, pose.get()))

        // only update if new data
        if (java.lang.Double.isFinite(inputs.timestamp)) {
            newFuel.clear()

            var i = 0
            while (i < driveHistory.size - 2 && driveHistory[i + 1].first > inputs.timestamp) {
                i++
            }
            val drivePos = driveHistory[i].second

            for (o in inputs.observations) {
                val d = (CAMERA_CONFIG.robotToCam.z - FUEL_RADIUS.`in`(Meters)) /
                    Math.tan(CAMERA_CONFIG.robotToCam.rotation.y - o.screenPos().y()) /
                    Math.cos(o.screenPos().x())

                // ignore fuel out of bounds
                if (d > MAX_DETECTION_DIST.`in`(Meters) || d < MIN_DETECTION_DIST.`in`(Meters)) {
                    continue
                }

                newFuel.add(
                    fuelData(
                        toFieldRelative(
                            Translation2d(
                                d * Math.cos(-CAMERA_CONFIG.robotToCam.rotation.x - o.screenPos().x()),
                                d * Math.sin(-CAMERA_CONFIG.robotToCam.rotation.x - o.screenPos().x())
                            ),
                            drivePos
                        ),
                        (inputs.timestamp * 1000000).toLong()
                    )
                )
            }

            clearFuelInFov(drivePos)
            clearFuelInRobot(drivePos)

            newFuel.forEach { addToMap(it) }
        }
        updateFuelPermanence()

        deduplicateFuelMap()

        Logger.recordOutput(
            "Subsystems/Fuel Vision/current detected fuel",
            *newFuel.stream().map { e -> Translation3d(e.pos.x, e.pos.y, FUEL_RADIUS.`in`(Meters)) }
                .toArray { size -> arrayOfNulls<Translation3d>(size) }
                .filterNotNull().toTypedArray()
        )
        Logger.recordOutput(
            "Subsystems/Fuel Vision/all mapped fuel",
            *getTrackedFuel().stream()
                .map { e -> Translation3d(e.x, e.y, FUEL_RADIUS.`in`(Meters)) }
                .toArray { size -> arrayOfNulls<Translation3d>(size) }
                .filterNotNull().toTypedArray()
        )

        // trajectory to render fov
        run {
            Logger.recordOutput(
                "Subsystems/Fuel Vision/fov",
                *arrayOf(
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 2)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 2)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 2.5)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 3)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 3.5)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 4)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 5)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 6)))
                        .plus(pose.get().translation),
                    // center
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation)
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 6)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 5)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 4)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 3.5)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 3)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 2.5)))
                        .plus(pose.get().translation),
                    Translation2d(MAX_CLEAR_DETECTION_DIST.`in`(Meters), 0.0) // other edge of fov
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 2)))
                        .plus(pose.get().translation),
                    // inner rad
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0) // other edge of fov
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 2)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 2.5)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 3)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 3.5)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 4)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 5)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.plus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 6)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation)
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 6)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 5)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 4)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 3.5)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 3)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0)
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 2.5)))
                        .plus(pose.get().translation),
                    Translation2d(MIN_CLEAR_DETECTION_DIST.`in`(Meters), 0.0) // one edge of fov
                        .rotateBy(pose.get().rotation.minus(Rotation2d(CAMERA_CLEAR_HFOV.`in`(Radians) / 2)))
                        .plus(pose.get().translation)
                )
            )
        }

        fuelMap.forEach { (coord, set) ->
            if (set == null || set.isEmpty()) {
                return@forEach
            }
            for (i in 0 until set.size) {
                cellsToRender.add(Translation3d(coord.x * GRID_SIZE.`in`(Meters), coord.y * GRID_SIZE.`in`(Meters), 0.5 * i))
            }
        }

        Logger.recordOutput("Subsystems/Fuel vision/cell density map", *cellsToRender.toTypedArray())
        cellsToRender.clear()
    }

    private fun deduplicateFuelMap() {
        // for each cell
        for (i in 0 until fuelMap.size) {
            @Suppress("UNCHECKED_CAST")
            val data = (fuelMap.values.toTypedArray() as Array<MutableSet<fuelData>>)[i].toTypedArray()
            // for each fuel in the cell
            for (j in data.indices) {
                val fuel = data[j]
                val ourCoord = getGridCoord(fuel.pos)
                // for all nearby cells, including this one
                for (dx in -1..1) {
                    for (dy in -1..1) {
                        val gridData = fuelMap[gridCoord(ourCoord.x + dx, ourCoord.y + dy)]
                        if (gridData == null || gridData.isEmpty()) {
                            continue // cell does not have data or does not exist
                        }
                        // remove if overlapping
                        gridData.removeIf { otherFuel -> fuel.pos.getDistance(otherFuel.pos) < FUEL_OVERLAP_THRESH.`in`(Meters) && otherFuel != fuel }
                    }
                }
            }
        }
    }

    private fun clearFuelInFov(pos: Pose2d) {
        fuelMap.forEach { (_, set) -> set.removeIf { oldTrack -> isInCameraFOV(oldTrack.pos(), pos) && Logger.getTimestamp() - oldTrack.timestamp > FUEL_POV_CLEAR_GRACE_TIME.`in`(Microseconds) } }
    }

    private fun clearFuelInRobot(pos: Pose2d) {
        val ourCoord = getGridCoord(pos.translation)
        // for all nearby cells, including this one
        for (dx in -1..1) {
            for (dy in -1..1) {
                val gridData = fuelMap[gridCoord(ourCoord.x + dx, ourCoord.y + dy)]
                if (gridData == null || gridData.isEmpty()) {
                    continue // cell does not have data or does not exist
                }
                // remove if overlapping
                gridData.removeIf { otherFuel ->
                    val fuelPos = otherFuel.pos().minus(pos.translation).rotateBy(pos.rotation)
                    fuelPos.x < ROBOT_WIDTH.`in`(Meters) / 2.0 &&
                        fuelPos.x > -ROBOT_WIDTH.`in`(Meters) / 2.0 &&
                        fuelPos.y < ROBOT_WIDTH.`in`(Meters) / 2.0 &&
                        fuelPos.y > -ROBOT_WIDTH.`in`(Meters) / 2.0
                }
            }
        }
    }

    /**
     * removes Object o such that {@code Objects.equals(o,fuel)} from the calculated gridCoord from {@code fuel.pos()}
     * @param fuel fuelData object to remove
     * @return true if successful, false otherwise
     */
    private fun removeFromMap(fuel: fuelData): Boolean {
        val coord = getGridCoord(fuel.pos)
        if (fuelMap.containsKey(coord)) {
            return fuelMap[coord]!!.remove(fuel)
        }
        return false
    }

    private fun isInCameraFOV(fieldPos: Translation2d, robotPose: Pose2d): Boolean {
        val cameraOffset = CAMERA_CONFIG.robotToCam.translation.toTranslation2d().rotateBy(robotPose.rotation)
        val cameraPos = robotPose.translation.plus(cameraOffset)
        val cameraYaw = robotPose.rotation.plus(Rotation2d(CAMERA_CONFIG.robotToCam.rotation.z))

        val distance = cameraPos.getDistance(fieldPos)
        if (distance > MAX_CLEAR_DETECTION_DIST.`in`(Meters) || distance < MIN_CLEAR_DETECTION_DIST.`in`(Meters)) {
            return false
        }

        val angleError = Math.abs(
            MathUtil.angleModulus(
                ExtraMath.getAngleToPos(fieldPos, cameraPos).`in`(Radians) - cameraYaw.radians
            )
        )
        return angleError <= CAMERA_CLEAR_HFOV.`in`(Radians) / 2.0
    }

    private fun updateFuelPermanence() {
        val now = Logger.getTimestamp()
        fuelMap.forEach { (_, set) ->
            set.removeIf { fuel -> Microseconds.of((now - fuel.timestamp).toDouble()).gte(FUEL_PERSISTANCE_TIME) }
        }
    }

    private fun addToMap(data: fuelData) {
        val mapCell = fuelMap.computeIfAbsent(getGridCoord(data.pos)) { HashSet() }
        mapCell.add(data)
    }

    private fun getGridCoord(fieldCoord: Translation2d): gridCoord {
        val coordx = Math.floor(fieldCoord.x / GRID_SIZE.`in`(Meters)).toInt()
        val coordy = Math.floor(fieldCoord.y / GRID_SIZE.`in`(Meters)).toInt()
        return gridCoord(coordx, coordy)
    }

    private fun toFieldRelative(pos: Translation2d): Translation2d {
        val cameraOffset = CAMERA_CONFIG.robotToCam.translation.toTranslation2d().rotateBy(pose.get().rotation)
        return pos
            .rotateBy(Rotation2d(pose.get().rotation.radians))
            .plus(Translation2d(pose.get().translation.x, pose.get().translation.y))
            .plus(cameraOffset)
    }

    private fun toFieldRelative(pos: Translation2d, drivePos: Pose2d): Translation2d {
        val cameraOffset = CAMERA_CONFIG.robotToCam.translation.toTranslation2d().rotateBy(drivePos.rotation)
        return pos
            .rotateBy(Rotation2d(drivePos.rotation.radians))
            .plus(Translation2d(drivePos.translation.x, drivePos.translation.y))
            .plus(cameraOffset)
    }

    fun getNewFuel(): ArrayList<Translation2d> {
        return newFuel.stream().map { it.pos }.collect(Collectors.toCollection { ArrayList() })
    }

    fun getTrackedFuel(): ArrayList<Translation2d> {
        return fuelMap.values.stream().flatMap { it.stream() }.map { it.pos }.collect(Collectors.toCollection { ArrayList() })
    }

    fun getFuelMap(): Map<gridCoord, MutableSet<fuelData>> {
        return fuelMap
    }

    data class gridCoord(val x: Int, val y: Int) {
        fun x(): Int = x
        fun y(): Int = y
    }

    data class fuelData(val pos: Translation2d, val timestamp: Long) {
        fun pos(): Translation2d = pos
        fun timestamp(): Long = timestamp
    }
}
