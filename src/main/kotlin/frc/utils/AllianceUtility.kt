package frc.utils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
object AllianceUtility {
    @JvmStatic
    private var latestAlliance: Alliance = Alliance.Blue

    private const val FIELD_LENGTH_METERS = 16.54
    private const val FIELD_WIDTH_METERS = 8.07

    @JvmField
    val FIELD_CENTER_POINT: Pose2d = Pose2d(FIELD_LENGTH_METERS / 2.0, FIELD_WIDTH_METERS / 2.0, Rotation2d.kZero)

    @JvmStatic
    fun getAlliance(): Alliance = latestAlliance

    @JvmStatic
    fun flipPose(original: Pose2d): Pose2d {
        if (latestAlliance == Alliance.Red) {
            return Pose2d(
                FIELD_CENTER_POINT.x + (FIELD_CENTER_POINT.x - original.x),
                FIELD_CENTER_POINT.y + (FIELD_CENTER_POINT.y - original.y),
                original.rotation.rotateBy(Rotation2d.k180deg)
            )
        }
        return original
    }

    @JvmStatic
    fun flipPose(vararg original: Pose2d): Array<Pose2d> {
        return original.map { flipPose(it) }.toTypedArray()
    }

    @JvmStatic
    fun flipPose(original: Translation3d): Translation3d {
        if (latestAlliance == Alliance.Red) {
            return Translation3d(
                FIELD_CENTER_POINT.x + (FIELD_CENTER_POINT.x - original.x),
                FIELD_CENTER_POINT.y + (FIELD_CENTER_POINT.y - original.y),
                original.z
            )
        }
        return original
    }

    @JvmStatic
    fun flipRectZone(original: RectZone): RectZone {
        if (latestAlliance == Alliance.Red) {
            return RectZone(
                FIELD_CENTER_POINT.x + (FIELD_CENTER_POINT.x - original.minX),
                FIELD_CENTER_POINT.y + (FIELD_CENTER_POINT.y - original.minY),
                FIELD_CENTER_POINT.x + (FIELD_CENTER_POINT.x - original.maxX),
                FIELD_CENTER_POINT.y + (FIELD_CENTER_POINT.y - original.maxY)
            )
        }
        return original
    }

    @JvmStatic
    fun forceFlipRectZone(original: RectZone): RectZone {
        return RectZone(
            FIELD_CENTER_POINT.x + (FIELD_CENTER_POINT.x - original.minX),
            FIELD_CENTER_POINT.y + (FIELD_CENTER_POINT.y - original.minY),
            FIELD_CENTER_POINT.x + (FIELD_CENTER_POINT.x - original.maxX),
            FIELD_CENTER_POINT.y + (FIELD_CENTER_POINT.y - original.maxY)
        )
    }

    @JvmStatic
    fun update() {
        latestAlliance = DriverStation.getAlliance().orElse(Alliance.Blue)
    }
}
