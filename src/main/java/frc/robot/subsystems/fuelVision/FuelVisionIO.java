package frc.robot.subsystems.fuelVision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public interface FuelVisionIO {
    
    public void updateInputs(FuelVisionIOInputs inputs);

    @AutoLog
    class FuelVisionIOInputs{
        public FuelObservation[] observations = {new FuelObservation(new Translation2d(), 0.0, 0.0, 0.0)};
        public Translation3d[] radialEstimates = new Translation3d[0];//estimates using estimated distance and angle to robot
        public Translation3d[] intersectionEstimates = new Translation3d[0];//estimates based on spherecast from cam to fuel
        public boolean connected = false;
    }
}
record FuelObservation(Translation2d screenPos, double area, double confidance, double dist){}