package frc.robot.subsystems.vision

import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import org.littletonrobotics.junction.AutoLog

interface CameraIO {
    fun updateInputs(inputs: CameraIOInputs) {}

    @AutoLog
    open class CameraIOInputs {
        @JvmField var connected: Boolean = false
        @JvmField var poseObservations: Array<PoseObservation> = emptyArray()
        @JvmField var tagIds: IntArray = IntArray(0)
        @JvmField var latestTargetObservation: TargetObservation = TargetObservation(Rotation2d(), Rotation2d(), -1)
        @JvmField var targets: Array<TargetObservation> = emptyArray()
        @JvmField var robotToCamera: Transform3d = Transform3d()
        @JvmField var name: String = ""
    }
}
