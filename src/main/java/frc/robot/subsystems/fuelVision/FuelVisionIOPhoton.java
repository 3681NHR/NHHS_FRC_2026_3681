package frc.robot.subsystems.fuelVision;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.VisionConstants.CameraConfig;

public class FuelVisionIOPhoton implements FuelVisionIO{
    private PhotonCamera cam;

    public FuelVisionIOPhoton(CameraConfig config){
        cam = new PhotonCamera(config.name);
    }
    
    public void updateInputs(FuelVisionIOInputs inputs){
        
        // public fuelObservation[] observations = {new fuelObservation(new Translation2d(), new Translation2d(), 0.0, 0.0)};
        // public Translation3d[] radialEstimates = new Translation3d[0];//estimates using estimated distance and angle to robot
        // public Translation3d[] intersectionEstimates = new Translation3d[0];//estimates based on spherecast from cam to fuel
        // public boolean connected = false;

        // public Transform3d robotToCamera = new Transform3d();
        // public String name = "";
    }
}