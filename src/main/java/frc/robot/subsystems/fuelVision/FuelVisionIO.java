package frc.robot.subsystems.fuelVision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

public interface FuelVisionIO {
    
    public void updateInputs(FuelVisionIOInputs inputs);

    @AutoLog
    class FuelVisionIOInputs{
        public fuelObservation[] observations = {new fuelObservation(new Translation2d(), new Translation2d(), 0.0, 0.0, 0.0)};
        public Translation3d[] radialEstimates = new Translation3d[0];//estimates using estimated distance and angle to robot
        public Translation3d[] intersectionEstimates = new Translation3d[0];//estimates based on spherecast from cam to fuel
        public boolean connected = false;

        public Transform3d robotToCamera = new Transform3d();
        public String name = "";
    }
}
record fuelObservation(Translation2d screenPos, Translation2d screensize, double area, double confidance, double stdDev){}