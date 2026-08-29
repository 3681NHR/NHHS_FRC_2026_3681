package frc.robot.subsystems.fuelVision;

public record FuelObservation(RadialPos2d screenPos, ScreenSize2d screensize, double area, double confidance, double dist) {}
