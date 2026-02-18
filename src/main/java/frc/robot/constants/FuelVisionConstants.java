package frc.robot.constants;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.VisionConstants.CameraConfig;

public class FuelVisionConstants {

    // Camera names, must match names configured on coprocessor
    public static CameraConfig CAMERA_CONFIG = new CameraConfig(
            "det",
            "object detector",
            1.0,
            1.0,
            new Transform3d(
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                Units.inchesToMeters(0),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(10),
                    Units.degreesToRadians(0)
                )
            )
        );
    // Basic filtering thresholds
    public static final double MAX_AMBIGUITY = 0.75;

    public static final Distance FUEL_RADIUS = Inches.of(3);

    //area of fuel on screen at 1 meter
    public static final double FUEL_SIZE_BASELINE = 0.1;

}