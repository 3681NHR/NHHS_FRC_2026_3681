package frc.robot.subsystems.fuelVision;

import static edu.wpi.first.units.Units.Radians;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;

public interface FuelVisionIO {
    
    public default void updateInputs(FuelVisionIOInputs inputs){}

    @AutoLog
    class FuelVisionIOInputs{
        public FuelObservation[] observations = {new FuelObservation(new RadialPos2d(0,0), new ScreenSize2d(0,0), 0.0, 0.0, 0.0)};
        public boolean connected = false;
    }
}
record FuelObservation(RadialPos2d screenPos, ScreenSize2d screensize, double area, double confidance, double dist){}
record RadialPos2d(double x, double y){}
record ScreenSize2d(double x, double y){}