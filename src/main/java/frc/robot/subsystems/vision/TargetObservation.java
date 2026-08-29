package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;

public record TargetObservation(Rotation2d tx, Rotation2d ty, int ID) {}
