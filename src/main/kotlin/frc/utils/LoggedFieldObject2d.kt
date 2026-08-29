package frc.utils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.trajectory.Trajectory
import edu.wpi.first.networktables.DoubleArrayEntry
import java.util.ArrayList
import java.util.Collections

class LoggedFieldObject2d internal constructor(
    @JvmField var m_name: String
) : AutoCloseable {

    @JvmField var m_entry: DoubleArrayEntry? = null
    private val m_poses: MutableList<Pose2d> = ArrayList()

    override fun close() {
        m_entry?.close()
    }

    @Synchronized
    fun setPose(pose: Pose2d) {
        setPoses(pose)
    }

    @Synchronized
    fun setPose(xMeters: Double, yMeters: Double, rotation: Rotation2d) {
        setPose(Pose2d(xMeters, yMeters, rotation))
    }

    @Synchronized
    fun getPose(): Pose2d {
        updateFromEntry()
        if (m_poses.isEmpty()) {
            return Pose2d.kZero
        }
        return m_poses[0]
    }

    @Synchronized
    fun setPoses(poses: List<Pose2d>) {
        m_poses.clear()
        m_poses.addAll(poses)
        updateEntry()
    }

    @Synchronized
    fun setPoses(vararg poses: Pose2d) {
        m_poses.clear()
        Collections.addAll(m_poses, *poses)
        updateEntry()
    }

    @Synchronized
    fun setTrajectory(trajectory: Trajectory) {
        m_poses.clear()
        for (state in trajectory.states) {
            m_poses.add(state.poseMeters)
        }
        updateEntry()
    }

    @Synchronized
    fun getPoses(): List<Pose2d> {
        updateFromEntry()
        return ArrayList(m_poses)
    }

    fun updateEntry() {
        updateEntry(false)
    }

    @Synchronized
    fun updateEntry(setDefault: Boolean) {
        val entry = m_entry ?: return
        val arr = DoubleArray(m_poses.size * 3)
        var ndx = 0
        for (pose in m_poses) {
            val translation: Translation2d = pose.translation
            arr[ndx + 0] = translation.x
            arr[ndx + 1] = translation.y
            arr[ndx + 2] = pose.rotation.degrees
            ndx += 3
        }
        if (setDefault) {
            entry.setDefault(arr)
        } else {
            entry.set(arr)
        }
    }

    @Synchronized
    private fun updateFromEntry() {
        val entry = m_entry ?: return
        val arr = entry.get(null) ?: return
        if (arr.size % 3 != 0) {
            return
        }
        m_poses.clear()
        var i = 0
        while (i < arr.size) {
            m_poses.add(Pose2d(arr[i], arr[i + 1], Rotation2d.fromDegrees(arr[i + 2])))
            i += 3
        }
    }
}
