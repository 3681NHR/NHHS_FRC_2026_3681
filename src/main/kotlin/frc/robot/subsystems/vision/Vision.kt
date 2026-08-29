package frc.robot.subsystems.vision

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.VecBuilder
import edu.wpi.first.math.geometry.Pose3d
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.VisionConstants.ANG_STD_DEV_BASELINE
import frc.robot.constants.VisionConstants.CAMERA_CONFIGS
import frc.robot.constants.VisionConstants.LIN_STD_DEV_BASELINE
import frc.robot.constants.VisionConstants.MAX_AMBIGUITY
import java.util.LinkedList
import java.util.Optional
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

class Vision(
    private var layout: AprilTagFieldLayout,
    vararg io: CameraIO
) : SubsystemBase() {

    private val io: Array<out CameraIO> = io
    private val inputs: Array<CameraIOInputsAutoLogged>
    private val disconnectedAlerts: Array<Alert>
    private val disabledAlerts: Array<Alert>
    private val disableCams: Array<LoggedNetworkBoolean>

    private var latestEstimateRaw: Array<VisionEstimate>? = null
    private var latestEstimateFinal: Array<VisionEstimate>? = latestEstimateRaw

    var estimates: MutableList<VisionEstimate> = LinkedList()
    var tagPoses: MutableList<Pose3d> = LinkedList()
    var robotPoses: MutableList<Pose3d> = LinkedList()
    var robotPosesAccepted: MutableList<Pose3d> = LinkedList()
    var robotPosesRejected: MutableList<Pose3d> = LinkedList()

    var allTagPoses: MutableList<Pose3d> = LinkedList()
    var allRobotPoses: MutableList<Pose3d> = LinkedList()
    var allRobotPosesAccepted: MutableList<Pose3d> = LinkedList()
    var allRobotPosesRejected: MutableList<Pose3d> = LinkedList()
    var allEstimates: MutableList<VisionEstimate> = LinkedList()

    init {
        inputs = Array(this.io.size) { CameraIOInputsAutoLogged() }
        disconnectedAlerts = Array(this.io.size) { Alert("", Alert.AlertType.kError) }
        disabledAlerts = Array(this.io.size) { Alert("", Alert.AlertType.kError) }
        disableCams = Array(this.io.size) { LoggedNetworkBoolean("", false) }
        for (i in this.io.indices) {
            this.io[i].updateInputs(inputs[i])
            Logger.processInputs("IO/Vision/" + CAMERA_CONFIGS[i].name, inputs[i])
            @Suppress("SENSELESS_COMPARISON")
            disconnectedAlerts[i] = Alert(
                if ("Camera: " + inputs[i].name == null) Integer.toString(i) else inputs[i].name + " is disconnected.",
                Alert.AlertType.kError
            )
            @Suppress("SENSELESS_COMPARISON")
            disabledAlerts[i] = Alert(
                if ("Camera: " + inputs[i].name == null) Integer.toString(i) else inputs[i].name + " is disabled.",
                Alert.AlertType.kError
            )
            @Suppress("SENSELESS_COMPARISON")
            disableCams[i] = LoggedNetworkBoolean(
                "Overrides/Vision/disable Camera: " + if (inputs[i].name == null) Integer.toString(i) else inputs[i].name,
                false
            )
        }
    }

    override fun periodic() {
        for (i in io.indices) {
            io[i].updateInputs(inputs[i])
            Logger.processInputs("IO/Vision/" + CAMERA_CONFIGS[i].name, inputs[i])
        }

        // Loop over cameras
        for (cameraIndex in io.indices) {
            if (!disableCams[cameraIndex].get()) {

                // Update disconnected alert
                disconnectedAlerts[cameraIndex].set(!inputs[cameraIndex].connected)
                disabledAlerts[cameraIndex].set(false)

                // Add tag poses
                for (tagId in inputs[cameraIndex].tagIds) {
                    val tagPose = layout.getTagPose(tagId)
                    if (tagPose.isPresent) {
                        tagPoses.add(tagPose.get())
                    }
                }

                // Loop over pose observations
                for (observation in inputs[cameraIndex].poseObservations) {
                    // Check whether to reject pose
                    val rejectPose = observation.tagCount() == 0 // Must have at least one tag
                        || observation.ambiguity() > MAX_AMBIGUITY // Cannot be too high ambiguity
                        || observation.pose().x < -1.0
                        || observation.pose().x > layout.fieldLength + 1
                        || observation.pose().y < -1.0
                        || observation.pose().y > layout.fieldWidth + 1

                    // Add pose to log
                    robotPoses.add(observation.pose())
                    if (rejectPose) {
                        robotPosesRejected.add(observation.pose())
                    } else {
                        robotPosesAccepted.add(observation.pose())
                    }

                    // Skip if rejected
                    if (rejectPose) {
                        continue
                    }

                    val stdDevFactor = Math.pow(observation.averageTagDistance(), 3.0) / observation.tagCount()
                    var linearStdDev = LIN_STD_DEV_BASELINE * stdDevFactor
                    var angularStdDev = ANG_STD_DEV_BASELINE * stdDevFactor
                    if (cameraIndex < CAMERA_CONFIGS.size) {
                        linearStdDev *= CAMERA_CONFIGS[cameraIndex].linStdDevFactor
                        angularStdDev *= CAMERA_CONFIGS[cameraIndex].angStdDevFactor
                    } else {
                        throw RuntimeException("could not find std dev factors for camera index: $cameraIndex")
                    }

                    estimates.add(
                        VisionEstimate(
                            observation.pose().toPose2d(),
                            observation.timestamp(),
                            VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev)
                        )
                    )
                }

                // Log camera data
                if (tagPoses.size > 0) {
                    Logger.recordOutput(
                        "Subsystems/Vision/Cameras/Camera: " + if (inputs[cameraIndex].name == null) Integer.toString(cameraIndex) else inputs[cameraIndex].name + "/TagPoses",
                        *tagPoses.toTypedArray()
                    )
                }
                if (robotPoses.size > 0) {
                    Logger.recordOutput(
                        "Subsystems/Vision/Cameras/Camera: " + if (inputs[cameraIndex].name == null) Integer.toString(cameraIndex) else inputs[cameraIndex].name + "/AllRobotPoses",
                        *robotPoses.toTypedArray()
                    )
                }
                if (robotPosesAccepted.size > 0) {
                    Logger.recordOutput(
                        "Subsystems/Vision/Cameras/Camera: " + if (inputs[cameraIndex].name == null) Integer.toString(cameraIndex) else inputs[cameraIndex].name + "/RobotPosesAccepted",
                        *robotPosesAccepted.toTypedArray()
                    )
                }
                if (robotPosesRejected.size > 0) {
                    Logger.recordOutput(
                        "Subsystems/Vision/Cameras/Camera: " + if (inputs[cameraIndex].name == null) Integer.toString(cameraIndex) else inputs[cameraIndex].name + "/RobotPosesRejected",
                        *robotPosesRejected.toTypedArray()
                    )
                }
                val stdDevs = Array(estimates.size) { DoubleArray(3) }
                for (i in stdDevs.indices) {
                    stdDevs[i] = estimates[i].visionMeasurementStdDevs!!.data
                }
                Logger.recordOutput(
                    "Subsystems/Vision/Cameras/Camera: " + if (inputs[cameraIndex].name == null) Integer.toString(cameraIndex) else inputs[cameraIndex].name + "/stdDevs",
                    stdDevs
                )

                allTagPoses.addAll(tagPoses)
                allRobotPoses.addAll(robotPoses)
                allRobotPosesAccepted.addAll(robotPosesAccepted)
                allRobotPosesRejected.addAll(robotPosesRejected)
                allEstimates.addAll(estimates)

                tagPoses.clear()
                robotPoses.clear()
                robotPosesAccepted.clear()
                robotPosesRejected.clear()
                estimates.clear()

            } else {
                disabledAlerts[cameraIndex].set(true)
            }
        }

        // Log summary data
        Logger.recordOutput("Subsystems/Vision/Summary/TagPoses", *allTagPoses.toTypedArray())
        Logger.recordOutput("Subsystems/Vision/Summary/tags", allTagPoses.size)
        Logger.recordOutput("Subsystems/Vision/Summary/RobotPoses", *allRobotPoses.toTypedArray())
        Logger.recordOutput("Subsystems/Vision/Summary/RobotPosesAccepted", *allRobotPosesAccepted.toTypedArray())
        Logger.recordOutput("Subsystems/Vision/Summary/RobotPosesRejected", *allRobotPosesRejected.toTypedArray())
        val stdDevs = Array(allEstimates.size) { DoubleArray(3) }
        for (i in stdDevs.indices) {
            stdDevs[i] = allEstimates[i].visionMeasurementStdDevs!!.data
        }
        Logger.recordOutput("Subsystems/Vision/Summary/stdDevs", stdDevs)

        latestEstimateRaw = allEstimates.toTypedArray()
        latestEstimateFinal = latestEstimateRaw

        allTagPoses.clear()
        allRobotPoses.clear()
        allRobotPosesAccepted.clear()
        allRobotPosesRejected.clear()
        allEstimates.clear()
    }

    fun getPose(): Array<VisionEstimate>? {
        // Send vision observation
        return latestEstimateFinal
    }

    fun getYaw(tagID: Int): Optional<Double> {
        var yaw: Optional<Double> = Optional.empty()
        if (tagID < 0) {
            for (i in inputs.indices) {
                yaw = Optional.of(inputs[i].latestTargetObservation.tx().radians + inputs[i].robotToCamera.rotation.z)
            }
        } else {
            for (i in inputs.indices) {
                for (t in inputs[i].targets) {
                    if (t != null) {
                        if (t.ID() == tagID) {
                            yaw = Optional.of(t.tx().radians + inputs[i].robotToCamera.rotation.z)
                        }
                    }
                }
            }
        }
        return yaw
    }
}
