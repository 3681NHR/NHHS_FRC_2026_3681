package frc.utils

import edu.wpi.first.math.geometry.Pose2d

object ZoneManager {
    @JvmStatic
    private var robotPose: Pose2d = Pose2d()

    private val hub: RectZone = RectZone(-0.5, -0.5, 4.0, 8.07)
    private val passing: RectZone = RectZone(5.2, 0.0, 16.5, 8.07)
    private val leftTrench: RectZone = RectZone(3.8, 6.87, 5.3, 8.07)
    private val rightTrench: RectZone = RectZone(3.8, 0.0, 5.3, 1.25)

    enum class FieldZone {
        HUB,
        PASS,
        TRENCH,
        UNKNOWN
    }

    @JvmStatic
    fun updateRobotPose(newRobotPose: Pose2d) {
        robotPose = newRobotPose
    }

    @JvmStatic
    fun getZone(): FieldZone {
        if (inLeftTrenchZone() || inRightTrenchZone()) return FieldZone.TRENCH
        if (inHubZone()) return FieldZone.HUB
        if (inPassingZone()) return FieldZone.PASS
        return FieldZone.UNKNOWN
    }

    private fun inHubZone(): Boolean =
        AllianceUtility.flipRectZone(hub).contains(robotPose.translation)

    private fun inPassingZone(): Boolean =
        AllianceUtility.flipRectZone(passing).contains(robotPose.translation)

    private fun inLeftTrenchZone(): Boolean =
        AllianceUtility.forceFlipRectZone(leftTrench).contains(robotPose.translation) ||
            leftTrench.contains(robotPose.translation)

    private fun inRightTrenchZone(): Boolean =
        AllianceUtility.forceFlipRectZone(rightTrench).contains(robotPose.translation) ||
            rightTrench.contains(robotPose.translation)
}
