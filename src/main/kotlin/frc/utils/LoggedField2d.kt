package frc.utils

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.networktables.NTSendable
import edu.wpi.first.networktables.NTSendableBuilder
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.util.sendable.SendableRegistry
import org.littletonrobotics.junction.Logger
import java.util.ArrayList

class LoggedField2d : NTSendable, AutoCloseable {

    private var m_table: NetworkTable? = null
    private val m_objects: MutableList<LoggedFieldObject2d> = ArrayList()

    init {
        val obj = LoggedFieldObject2d("Robot")
        obj.setPose(Pose2d.kZero)
        m_objects.add(obj)
        SendableRegistry.add(this, "Field")
    }

    override fun close() {
        for (obj in m_objects) {
            obj.close()
        }
    }

    @Synchronized
    fun setRobotPose(pose: Pose2d) {
        m_objects[0].setPose(pose)
    }

    @Synchronized
    fun setRobotPose(xMeters: Double, yMeters: Double, rotation: Rotation2d) {
        m_objects[0].setPose(xMeters, yMeters, rotation)
    }

    @Synchronized
    fun getRobotPose(): Pose2d = m_objects[0].getPose()

    @Synchronized
    fun getObject(name: String): LoggedFieldObject2d {
        for (obj in m_objects) {
            if (obj.m_name == name) {
                return obj
            }
        }
        val obj = LoggedFieldObject2d(name)
        m_objects.add(obj)
        val table = m_table
        if (table != null) {
            synchronized(obj) {
                obj.m_entry = table.getDoubleArrayTopic(name).getEntry(doubleArrayOf())
            }
        }
        return obj
    }

    @Synchronized
    fun getRobotObject(): LoggedFieldObject2d = m_objects[0]

    override fun initSendable(builder: NTSendableBuilder) {
        builder.setSmartDashboardType("Field2d")
        synchronized(this) {
            m_table = builder.table
            val table = m_table
            if (table != null) {
                for (obj in m_objects) {
                    synchronized(obj) {
                        obj.m_entry = table.getDoubleArrayTopic(obj.m_name).getEntry(doubleArrayOf())
                        obj.updateEntry(true)
                    }
                }
            }
        }
    }

    fun log(key: String) {
        for (obj in m_objects) {
            Logger.recordOutput("$key/${obj.m_name}", *obj.getPoses().toTypedArray())
        }
    }
}
