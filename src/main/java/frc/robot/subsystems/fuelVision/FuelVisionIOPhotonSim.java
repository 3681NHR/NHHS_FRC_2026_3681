package frc.robot.subsystems.fuelVision;

import java.util.ArrayList;
import java.util.function.Supplier;
import org.ironmaple.simulation.SimulatedArena;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.simulation.VisionTargetSim;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.constants.FuelVisionConstants;
import frc.robot.constants.VisionConstants.CameraConfig;

import static edu.wpi.first.units.Units.Degrees;
import static frc.robot.constants.FuelVisionConstants.CAMERA_CLEAR_HFOV;
import static frc.robot.constants.FuelVisionConstants.CAMERA_REAL_HFOV;

public class FuelVisionIOPhotonSim extends FuelVisionIOPhoton{
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    /**
     * Creates a new CameraIOPhotonSim.
     *
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public FuelVisionIOPhotonSim(
            CameraConfig config, Supplier<Pose2d> poseSupplier) {
        super(config);
        this.poseSupplier = poseSupplier;

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setFPS(30);
        cameraProperties.setAvgLatencyMs(45);
        cameraProperties.setLatencyStdDevMs(20);
        cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(CAMERA_REAL_HFOV.in(Degrees)));
        cameraProperties.setCalibError(0.1, 0.02);
        cameraSim = new PhotonCameraSim(cam, cameraProperties);
        visionSim.addCamera(cameraSim, config.robotToCam);
    }

    @Override
    public void updateInputs(FuelVisionIOInputs inputs) {
        visionSim.clearVisionTargets();
        ArrayList<Double> distances = new ArrayList<Double>();
        for(var f : SimulatedArena.getInstance().getGamePiecesByType("fuel")){
            visionSim.addVisionTargets(new VisionTargetSim(f.getPose3d(), new TargetModel(0.15)));
            distances.add(FuelVisionConstants.CAMERA_CONFIG.robotToCam.getTranslation()
            .rotateAround(new Translation3d(), new Rotation3d(0,0,poseSupplier.get().getRotation().getRadians()))
            .plus(new Translation3d(poseSupplier.get().getTranslation().getX(), poseSupplier.get().getTranslation().getY(), 0)).getDistance(f.getPose3d().getTranslation()));
        }
        org.littletonrobotics.junction.Logger.recordOutput("Subsystems/FuelVision/sim/real dist", distances.stream().mapToDouble(e->e).toArray());
        visionSim.update(poseSupplier.get());
        super.updateInputs(inputs);
    }
}