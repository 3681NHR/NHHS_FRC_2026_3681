package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.units.Units.Microseconds
import edu.wpi.first.units.Units.Seconds
import frc.robot.constants.VisionConstants.CameraConfig
import java.util.function.Supplier
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim

/** IO implementation for physics sim using PhotonVision simulator. */
class CameraIOPhotonSim(
    layout: AprilTagFieldLayout,
    config: CameraConfig,
    private val poseSupplier: Supplier<Pose2d>
) : CameraIOPhoton(layout, config) {

    private val cameraSim: PhotonCameraSim
    private val simulatePhoton = LoggedNetworkBoolean("Sim/simulate vision", false)

    init {
        if (visionSim == null) {
            visionSim = VisionSystemSim("main")
            visionSim!!.addAprilTags(layout)
        }
        val cameraProperties = SimCameraProperties()
        cameraProperties.setFPS(45.0)
        cameraProperties.setAvgLatencyMs(23.0)
        cameraProperties.setLatencyStdDevMs(20.0)
        cameraProperties.setCalibration(800, 600, Rotation2d.fromDegrees(110.0))
        cameraProperties.setCalibError(0.1, 0.02)
        cameraSim = PhotonCameraSim(camera, cameraProperties)
        visionSim!!.addCamera(cameraSim, config.robotToCam)
    }

    override fun updateInputs(inputs: CameraIO.CameraIOInputs) {
        if (simulatePhoton.getAsBoolean()) {
            visionSim!!.update(poseSupplier.get())
            super.updateInputs(inputs)
        } else {
            inputs.poseObservations = arrayOf(
                PoseObservation(
                    Microseconds.of(Logger.getTimestamp().toDouble()).`in`(Seconds),
                    Pose3d(
                        poseSupplier.get().x,
                        poseSupplier.get().y,
                        0.0,
                        Rotation3d(0.0, 0.0, poseSupplier.get().rotation.radians)
                    ),
                    -1.0,
                    1,
                    1.0
                )
            )
            inputs.connected = camera.isConnected
            inputs.name = camera.name
            inputs.robotToCamera = super.robotToCamera
            inputs.connected = true
        }
    }

    companion object {
        private var visionSim: VisionSystemSim? = null
    }
}
