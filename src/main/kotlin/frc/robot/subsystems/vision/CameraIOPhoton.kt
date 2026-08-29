package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform3d
import frc.robot.constants.VisionConstants.CameraConfig
import org.photonvision.PhotonCamera
import org.photonvision.PhotonPoseEstimator

open class CameraIOPhoton(
    layout: AprilTagFieldLayout,
    config: CameraConfig
) : CameraIO {

    protected val camera: PhotonCamera = PhotonCamera(config.name)
    private val poseEstimator: PhotonPoseEstimator = PhotonPoseEstimator(layout, config.robotToCam)

    @JvmField
    val robotToCamera: Transform3d = config.robotToCam

    override fun updateInputs(inputs: CameraIO.CameraIOInputs) {
        val observations = ArrayList<PoseObservation>()

        for (result in camera.allUnreadResults) {
            var estimate = poseEstimator.estimateCoprocMultiTagPose(result)
            if (!estimate.isPresent) {
                estimate = poseEstimator.estimateLowestAmbiguityPose(result)
            }
            if (estimate.isPresent) {
                observations.add(
                    PoseObservation(
                        estimate.get().timestampSeconds,
                        estimate.get().estimatedPose,
                        if (result.multitagResult.isPresent) result.multitagResult.get().estimatedPose.ambiguity else -1.0,
                        estimate.get().targetsUsed.size,
                        getAvgDistance(result)
                    )
                )
                inputs.tagIds = estimate.get().targetsUsed.stream().mapToInt { t -> t.fiducialId }.toArray()
                inputs.latestTargetObservation = TargetObservation(
                    Rotation2d(result.bestTarget.yaw),
                    Rotation2d(result.bestTarget.pitch),
                    result.bestTarget.fiducialId
                )
                inputs.targets = result.targets.map { t ->
                    TargetObservation(
                        Rotation2d(t.yaw),
                        Rotation2d(t.pitch),
                        t.fiducialId
                    )
                }.toTypedArray()
            } else {
                inputs.targets = emptyArray()
                inputs.tagIds = IntArray(0)
            }
        }
        inputs.connected = camera.isConnected
        inputs.poseObservations = observations.toTypedArray()
        inputs.name = camera.name
        inputs.robotToCamera = robotToCamera

        observations.clear()
    }

    fun getAvgDistance(res: org.photonvision.targeting.PhotonPipelineResult): Double {
        var sum = 0.0
        for (target in res.targets) {
            sum += target.bestCameraToTarget.translation.norm
        }
        return if (res.targets.isEmpty()) 0.0 else sum / res.targets.size
    }
}
