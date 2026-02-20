package frc.robot.subsystems.fuelVision;

import static edu.wpi.first.units.Units.Degrees;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.FuelVisionConstants;
import frc.robot.constants.VisionConstants.CameraConfig;
import frc.utils.ExtraMath;

public class FuelVisionIOPhoton implements FuelVisionIO{
    public PhotonCamera cam;
    
    ArrayList<FuelObservation> observations = new ArrayList<FuelObservation>();
    ArrayList<Translation3d> radialPositions = new ArrayList<Translation3d>();
    ArrayList<Translation3d> interceptPositions = new ArrayList<Translation3d>();

    public FuelVisionIOPhoton(CameraConfig config){
        cam = new PhotonCamera(config.name);
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
                    List<TargetCorner> corners = targ.getMinAreaRectCorners();
                    /*
                     *  0---1
                     *  |   |
                     *  2---3
                     */
                    assert(corners.size() == 4);
                    observations.add(new FuelObservation(
                        new RadialPos2d(Degrees.of(targ.yaw), Degrees.of(targ.pitch)),
                        new ScreenSize2d(
                            Math.abs(ExtraMath.max(corners.get(0).x, corners.get(1).x, corners.get(2).x, corners.get(3).x) - ExtraMath.min(corners.get(0).x, corners.get(1).x, corners.get(2).x, corners.get(3).x))/480.0,
                            Math.abs(ExtraMath.max(corners.get(0).y, corners.get(1).y, corners.get(2).y, corners.get(3).y) - ExtraMath.min(corners.get(0).y, corners.get(1).y, corners.get(2).y, corners.get(3).y))/640.0
                            ),
                        targ.area,
                        targ.objDetectConf,
                        FuelVisionConstants.FUEL_SIZE_BASELINE/targ.area
                    ));
                }
            }
        }
        inputs.observations = observations.toArray(new FuelObservation[0]);
        inputs.connected = cam.isConnected();

    }
}