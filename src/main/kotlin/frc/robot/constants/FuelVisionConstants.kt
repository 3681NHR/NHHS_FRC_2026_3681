package frc.robot.constants

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.math.util.Units
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Milliseconds
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Time
import frc.robot.constants.VisionConstants.CameraConfig

object FuelVisionConstants {

    @JvmField var CAMERA_CONFIG: CameraConfig = CameraConfig(
        "det",
        "object detector",
        1.0,
        1.0,
        Transform3d(
            Units.inchesToMeters(3.185501),
            Units.inchesToMeters(0.733652 - 1.0),
            Units.inchesToMeters(19.913505),
            Rotation3d(
                Units.degreesToRadians(0.0),
                Units.degreesToRadians(10.0),
                Units.degreesToRadians(0.0)
            )
        )
    )

    @JvmField val FUEL_RADIUS: Distance = Inches.of(3.0)

    const val FUEL_SIZE_BASELINE: Double = 1.54

    @JvmField val FUEL_PERSISTANCE_TIME: Time = Seconds.of(3.0)
    @JvmField val FUEL_POV_CLEAR_GRACE_TIME: Time = Milliseconds.of(200.0)
    @JvmField val FUEL_OVERLAP_THRESH: Distance = Inches.of(4.0)

    @JvmField val CAMERA_CLEAR_HFOV: Angle = Degrees.of(60.0)
    @JvmField val MIN_CLEAR_DETECTION_DIST: Distance = Meters.of(1.0)
    @JvmField val MAX_CLEAR_DETECTION_DIST: Distance = Meters.of(5.0)

    @JvmField val MIN_DETECTION_DIST: Distance = Meters.of(0.5)
    @JvmField val MAX_DETECTION_DIST: Distance = Meters.of(6.0)

    @JvmField val CAMERA_REAL_HFOV: Angle = Degrees.of(71.2)
    @JvmField val GRID_SIZE: Distance = Inches.of(20.0)

    @JvmField val ROBOT_WIDTH: Distance = Inches.of(20.0)

    const val MAX_FUEL: Int = 500
}
