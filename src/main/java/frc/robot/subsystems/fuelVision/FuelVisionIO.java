package frc.robot.subsystems.fuelVision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Translation2d;

public interface FuelVisionIO {
    
    public void updateInputs(FuelVisionIOInputs inputs);

    @AutoLog
    class FuelVisionIOInputs{
        public FuelObservation[] observations = {new FuelObservation(new Translation2d(), new Translation2d(), 0.0, 0.0, 0.0)};
        public boolean connected = false;
    }
}
record FuelObservation(Translation2d screenPos, Translation2d screensize, double area, double confidance, double dist){}