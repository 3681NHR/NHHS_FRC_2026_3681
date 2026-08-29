package frc.robot.subsystems.fuelVision

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Degrees
import frc.robot.constants.FuelVisionConstants
import frc.robot.constants.FuelVisionConstants.CAMERA_CLEAR_HFOV
import frc.robot.constants.FuelVisionConstants.CAMERA_REAL_HFOV
import frc.robot.constants.VisionConstants.CameraConfig
import java.util.ArrayList
import java.util.function.Supplier
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.Logger
import org.photonvision.estimation.TargetModel
import org.photonvision.simulation.PhotonCameraSim
import org.photonvision.simulation.SimCameraProperties
import org.photonvision.simulation.VisionSystemSim
import org.photonvision.simulation.VisionTargetSim

class FuelVisionIOPhotonSim(
    config: CameraConfig,
    private val poseSupplier: Supplier<Pose2d>
) : FuelVisionIOPhoton(config) {

    private val cameraSim: PhotonCameraSim

    init {
        if (visionSim == null) {
            visionSim = VisionSystemSim("main")
        }
        val cameraProperties = SimCameraProperties()
        cameraProperties.setFPS(30.0)
        cameraProperties.setAvgLatencyMs(45.0)
        cameraProperties.setLatencyStdDevMs(20.0)
        cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(CAMERA_REAL_HFOV.`in`(Degrees)))
        cameraProperties.setCalibError(0.1, 0.02)
        cameraSim = PhotonCameraSim(cam, cameraProperties)
        visionSim!!.addCamera(cameraSim, config.robotToCam)
    }

    override fun updateInputs(inputs: FuelVisionIO.FuelVisionIOInputs) {
        visionSim!!.clearVisionTargets()
        val distances = ArrayList<Double>()
        for (f in SimulatedArena.getInstance().getGamePiecesByType("fuel")) {
            visionSim!!.addVisionTargets(VisionTargetSim(f.pose3d, TargetModel(0.15)))
            distances.add(
                FuelVisionConstants.CAMERA_CONFIG.robotToCam.translation
                    .rotateAround(Translation3d(), Rotation3d(0.0, 0.0, poseSupplier.get().rotation.radians))
                    .plus(Translation3d(poseSupplier.get().translation.x, poseSupplier.get().translation.y, 0.0))
                    .getDistance(f.pose3d.translation)
            )
        }
        Logger.recordOutput("Subsystems/FuelVision/sim/real dist", distances.map { it }.toDoubleArray())
        visionSim!!.update(poseSupplier.get())
        super.updateInputs(inputs)
    }

    companion object {
        private var visionSim: VisionSystemSim? = null
    }
}
