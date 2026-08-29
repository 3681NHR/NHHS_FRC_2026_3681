package frc.robot.constants

import edu.wpi.first.apriltag.AprilTagFieldLayout
import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.measure.Distance

object VisionConstants {

    class CameraConfig {
        @JvmField var name: String
        @JvmField var dashboardName: String
        @JvmField var linStdDevFactor: Double
        @JvmField var angStdDevFactor: Double
        @JvmField var robotToCam: Transform3d

        constructor(name: String, dashboardName: String, linStdDevFactor: Double, angStdDevFactor: Double, robotToCam: Transform3d) {
            this.name = name
            this.dashboardName = dashboardName
            this.linStdDevFactor = linStdDevFactor
            this.angStdDevFactor = angStdDevFactor
            this.robotToCam = robotToCam
        }

        constructor(name: String, linStdDevFactor: Double, angStdDevFactor: Double, robotToCam: Transform3d) {
            this.name = name
            this.dashboardName = name
            this.linStdDevFactor = linStdDevFactor
            this.angStdDevFactor = angStdDevFactor
            this.robotToCam = robotToCam
        }
    }

    @JvmField var APRILTAG_LAYOUT: AprilTagFieldLayout? = null

    @JvmField var CAMERA_CONFIGS: Array<CameraConfig> = arrayOf(
        CameraConfig(
            "front",
            "front",
            1.0,
            1.0,
            Transform3d(
                Units.inchesToMeters(3.183792),
                Units.inchesToMeters(-4.016348),
                Units.inchesToMeters(20.312005),
                Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-10.0),
                    Units.degreesToRadians(0.0)
                )
            )
        ),
        CameraConfig(
            "right",
            "right",
            1.0,
            1.0,
            Transform3d(
                Units.inchesToMeters(-8.375000),
                Units.inchesToMeters(-12.871373),
                Units.inchesToMeters(8.188547),
                Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-30.0),
                    Units.degreesToRadians(-90.0)
                )
            )
        ),
        CameraConfig(
            "left",
            "left",
            1.0,
            1.0,
            Transform3d(
                Units.inchesToMeters(-8.375520),
                Units.inchesToMeters(12.871499),
                Units.inchesToMeters(8.188547),
                Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-30.0),
                    Units.degreesToRadians(90.0)
                )
            )
        ),
        CameraConfig(
            "back",
            "back",
            1.0,
            1.0,
            Transform3d(
                Units.inchesToMeters(-12.871499),
                Units.inchesToMeters(-8.375520),
                Units.inchesToMeters(8.188547),
                Rotation3d(
                    Units.degreesToRadians(0.0),
                    Units.degreesToRadians(-30.0),
                    Units.degreesToRadians(180.0)
                )
            )
        )
    )

    @JvmField var MAX_AMBIGUITY: Double = 0.75
    @JvmField var MAX_Z_ERROR: Distance = Meters.of(0.75)

    @JvmField var LIN_STD_DEV_BASELINE: Double = 0.2
    @JvmField var ANG_STD_DEV_BASELINE: Double = 0.1
}
