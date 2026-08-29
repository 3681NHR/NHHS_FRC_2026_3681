package frc.robot.subsystems.vision

import edu.wpi.first.math.Matrix
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.numbers.N1
import edu.wpi.first.math.numbers.N3

class VisionEstimate {
    @JvmField var pose: Pose2d
    @JvmField var timestampSeconds: Double
    @JvmField var visionMeasurementStdDevs: Matrix<N3, N1>?

    constructor(visionRobotPoseMeters: Pose2d, timestampSeconds: Double, visionMeasurementStdDevs: Matrix<N3, N1>?) {
        this.pose = visionRobotPoseMeters
        this.timestampSeconds = timestampSeconds
        this.visionMeasurementStdDevs = visionMeasurementStdDevs
    }

    constructor() {
        this.pose = Pose2d()
        this.timestampSeconds = 0.0
        this.visionMeasurementStdDevs = null
    }
}
