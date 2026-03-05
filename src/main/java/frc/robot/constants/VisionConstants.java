package frc.robot.constants;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
    /**
     * object to store camera configuration data
     */
    public static class CameraConfig{
        public String name;//name set in photonvision
        public String dashboardName;//name to display in dashboard
        public double linStdDevFactor;//standard deviation factor for linear distance, increse to trust less
        public double angStdDevFactor;//standard deviation factor for angular distance, increase to trust less
        public Transform3d robotToCam;//transform3d from robot center to camera, meters and radians

        public CameraConfig(String name, String dashboardName, double linStdDevFactor, double angStdDevFactor, Transform3d robotToCam){
            this.name = name;
            this.dashboardName = dashboardName;
            this.linStdDevFactor = linStdDevFactor;
            this.angStdDevFactor = angStdDevFactor;
            this.robotToCam = robotToCam;
        }
        public CameraConfig(String name, double linStdDevFactor, double angStdDevFactor, Transform3d robotToCam){
            this.name = name;
            this.dashboardName = name;
            this.linStdDevFactor = linStdDevFactor;
            this.angStdDevFactor = angStdDevFactor;
            this.robotToCam = robotToCam;
        }
    }
      // AprilTag layout on field, loaded from json file in robotcontainer
    public static AprilTagFieldLayout APRILTAG_LAYOUT;

    // Camera names, must match names configured on coprocessor
    public static CameraConfig[] CAMERA_CONFIGS = {
        new CameraConfig(
            "front",
            "front",
            1.0,
            1.0,
            new Transform3d(
                Units.inchesToMeters(3.183792),
                Units.inchesToMeters(-3.016348),
                Units.inchesToMeters(20.312005),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-10),
                    Units.degreesToRadians(0)
                )
            )
        ),
        new CameraConfig(
            "right",
            "right",
            1.0,
            1.0,
            new Transform3d(
                Units.inchesToMeters(-8.375000),
                Units.inchesToMeters(-12.871373),
                Units.inchesToMeters(8.188547),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-30),
                    Units.degreesToRadians(-90)
                )
            )
        ),
        new CameraConfig(
            "left",
            "left",
            1.0,
            1.0,
            new Transform3d(
                Units.inchesToMeters(-8.375520),
                Units.inchesToMeters(12.871499),
                Units.inchesToMeters(8.188547),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-30),
                    Units.degreesToRadians(90)
                )
            )
        ),
        new CameraConfig(
            "back",
            "back",
            1.0,
            1.0,
            new Transform3d(
                Units.inchesToMeters(-12.871499),
                Units.inchesToMeters(-8.375520),
                Units.inchesToMeters(8.188547),
                new Rotation3d(
                    Units.degreesToRadians(0),
                    Units.degreesToRadians(-30),
                    Units.degreesToRadians(180)
                )
            )
        )
    };
    // Basic filtering thresholds
    public static double MAX_AMBIGUITY = 0.75;
    public static Distance MAX_Z_ERROR = Meters.of(0.75);

    // Standard deviation baselines, for 1 meter distance and 1 tag
    // (Adjusted automatically based on distance and # of tags)
    public static double LIN_STD_DEV_BASELINE = 0.2; // Meters
    public static double ANG_STD_DEV_BASELINE = 0.1; // Radians

}