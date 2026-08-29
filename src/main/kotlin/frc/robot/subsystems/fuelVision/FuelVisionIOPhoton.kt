package frc.robot.subsystems.fuelVision

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.wpilibj.Alert
import frc.robot.constants.FuelVisionConstants
import frc.robot.constants.VisionConstants.CameraConfig
import frc.utils.ExtraMath
import org.photonvision.PhotonCamera

open class FuelVisionIOPhoton(
    config: CameraConfig
) : FuelVisionIO {

    @JvmField
    val cam: PhotonCamera = PhotonCamera(config.name)

    var observations: ArrayList<FuelObservation> = ArrayList()
    var radialPositions: ArrayList<Translation3d> = ArrayList()
    var interceptPositions: ArrayList<Translation3d> = ArrayList()

    private val disconnect = Alert("object detection cam is disconnected!", Alert.AlertType.kError)

    override fun updateInputs(inputs: FuelVisionIO.FuelVisionIOInputs) {
        val results = cam.allUnreadResults

        if (results.isNotEmpty()) {
            observations.clear()
            radialPositions.clear()
            interceptPositions.clear()
            val result = results[0]
            inputs.timestamp = result.timestampSeconds
            if (result.hasTargets()) {
                for (targ in result.targets) {
                    val corners = targ.minAreaRectCorners
                    /*
                     *  0---1
                     *  |   |
                     *  2---3
                     */
                    observations.add(
                        FuelObservation(
                            RadialPos2d(Units.degreesToRadians(targ.yaw), Units.degreesToRadians(targ.pitch)),
                            ScreenSize2d(
                                Math.abs(
                                    ExtraMath.max(corners[0].x.toDouble(), corners[1].x.toDouble(), corners[2].x.toDouble(), corners[3].x.toDouble()) -
                                        ExtraMath.min(corners[0].x.toDouble(), corners[1].x.toDouble(), corners[2].x.toDouble(), corners[3].x.toDouble())
                                ) / 480.0,
                                Math.abs(
                                    ExtraMath.max(corners[0].y.toDouble(), corners[1].y.toDouble(), corners[2].y.toDouble(), corners[3].y.toDouble()) -
                                        ExtraMath.min(corners[0].y.toDouble(), corners[1].y.toDouble(), corners[2].y.toDouble(), corners[3].y.toDouble())
                                ) / 640.0
                            ),
                            targ.area.toDouble(),
                            targ.objDetectConf.toDouble(),
                            FuelVisionConstants.FUEL_SIZE_BASELINE / targ.area.toDouble()
                        )
                    )
                }
            }
        }
        inputs.observations = observations.toTypedArray()
        inputs.connected = cam.isConnected
        disconnect.set(!inputs.connected)
    }
}
