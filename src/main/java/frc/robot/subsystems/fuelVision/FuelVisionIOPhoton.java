package frc.robot.subsystems.fuelVision;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLog;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.FuelVisionConstants;
import frc.robot.constants.VisionConstants.CameraConfig;

public class FuelVisionIOPhoton implements FuelVisionIO{
    private PhotonCamera cam;
    private Supplier<Pose2d> pose;
    
    ArrayList<FuelObservation> observations = new ArrayList<FuelObservation>();
    ArrayList<Translation3d> radialPositions = new ArrayList<Translation3d>();
    ArrayList<Translation3d> interceptPositions = new ArrayList<Translation3d>();

    public FuelVisionIOPhoton(CameraConfig config, Supplier<Pose2d> pose){
        cam = new PhotonCamera(config.name);
        this.pose = pose;
    }
    
    public void updateInputs(FuelVisionIOInputs inputs){
        var results = cam.getAllUnreadResults();

        if(results.size() > 0){
            observations.clear();
            radialPositions.clear();
            interceptPositions.clear();
            var result = results.get(0);
            if(result.hasTargets()){
                for(PhotonTrackedTarget targ : result.getTargets()){
                    observations.add(new FuelObservation(
                        new Translation2d(targ.yaw, targ.pitch),
                        targ.area,
                        targ.objDetectConf,
                        Math.sqrt(FuelVisionConstants.FUEL_SIZE_BASELINE/targ.area)
                    ));
                    double hDist = Math.cos(Units.degreesToRadians(targ.pitch))*Math.sqrt(FuelVisionConstants.FUEL_SIZE_BASELINE/targ.area);
                    double vDist = Math.sin(Units.degreesToRadians(targ.pitch))*Math.sqrt(FuelVisionConstants.FUEL_SIZE_BASELINE/targ.area);
                    radialPositions.add(new Translation3d(Math.sin(Units.degreesToRadians(targ.yaw))*hDist,Math.cos(Units.degreesToRadians(targ.yaw))*hDist,vDist));

                    double hDist2 = (FuelVisionConstants.CAMERA_CONFIG.robotToCam.getZ()-FuelVisionConstants.FUEL_RADIUS.in(Meters))/Math.tan(FuelVisionConstants.CAMERA_CONFIG.robotToCam.getRotation().getY()-Units.degreesToRadians(targ.pitch));
                    interceptPositions.add(new Translation3d(Math.sin(Units.degreesToRadians(targ.yaw))*hDist2,Math.cos(Units.degreesToRadians(targ.yaw))*hDist2,FuelVisionConstants.FUEL_RADIUS.in(Meters)));
                }
            }
        }
        inputs.observations = observations.toArray(new FuelObservation[0]);
        inputs.radialEstimates = radialPositions.stream()
            .map(pos -> pos.rotateAround(new Translation3d(), new Rotation3d(0,0,FuelVisionConstants.CAMERA_CONFIG.robotToCam.getRotation().getZ()).plus(new Rotation3d(0,0,pose.get().getRotation().getRadians())))
                            .plus(new Translation3d(getFieldPos().getX(), getFieldPos().getY(), FuelVisionConstants.CAMERA_CONFIG.robotToCam.getZ())))
            .toArray(Translation3d[]::new);
        inputs.intersectionEstimates = interceptPositions.stream()
            .map(pos -> pos.rotateAround(new Translation3d(), FuelVisionConstants.CAMERA_CONFIG.robotToCam.getRotation().plus(new Rotation3d(0,0,pose.get().getRotation().getRadians())))
                            .plus(new Translation3d(getFieldPos().getX(), getFieldPos().getY(), 0)))
            .toArray(Translation3d[]::new);
        inputs.connected = cam.isConnected();
        
        // public Translation3d[] intersectionEstimates = new Translation3d[0];//estimates based on spherecast from cam to fuel

    }
    private Translation2d getFieldPos(){
        if(pose.get() != null){
        return pose.get().getTranslation().plus(FuelVisionConstants.CAMERA_CONFIG.robotToCam.getTranslation().rotateAround(new Translation3d(), new Rotation3d(0,0,pose.get().getRotation().getRadians())).toTranslation2d());
        } else {
            return FuelVisionConstants.CAMERA_CONFIG.robotToCam.getTranslation().toTranslation2d();
        }
    }
}