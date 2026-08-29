package frc.utils

import edu.wpi.first.math.geometry.Translation2d

class RectZone(minX: Double, minY: Double, maxX: Double, maxY: Double) {
    @JvmField
    val minX: Double = minOf(minX, maxX)
    @JvmField
    val minY: Double = minOf(minY, maxY)
    @JvmField
    val maxX: Double = maxOf(minX, maxX)
    @JvmField
    val maxY: Double = maxOf(minY, maxY)

    fun contains(point: Translation2d): Boolean =
        point.x >= minX &&
            point.x <= maxX &&
            point.y >= minY &&
            point.y <= maxY
}
