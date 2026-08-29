package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;

public record PoseObservation(
        double timestamp,
        Pose3d pose,
        double ambiguity,
        int tagCount,
        double averageTagDistance) {}
