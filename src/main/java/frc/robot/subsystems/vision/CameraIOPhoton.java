package frc.robot.subsystems.vision;

import static edu.wpi.first.units.Units.Meters;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.VisionConstants.CameraConfig;

public class CameraIOPhoton implements CameraIO {

    protected final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    private final Transform3d robotToCamera;

    /**
     * Creates a new CameraIOPhoton.
     *
     * @param name             The configured name of the camera.
     * @param rotationSupplier The 3D position of the camera relative to the robot.
     */
    public CameraIOPhoton(AprilTagFieldLayout layout, CameraConfig config) {
        camera = new PhotonCamera(config.name);
        this.robotToCamera = config.robotToCam;

        this.poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        ArrayList<PoseObservation> observations = new ArrayList<>();

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> estimate = poseEstimator.update(result);
            if (estimate.isPresent()) {
                observations.add(new PoseObservation(
                        estimate.get().timestampSeconds,
                        estimate.get().estimatedPose,
                        result.multitagResult.isPresent() ? result.multitagResult.get().estimatedPose.ambiguity : -1,
                        estimate.get().targetsUsed.size(),
                        getAvgDistance(result)));
                inputs.tagIds = estimate.get().targetsUsed.stream().mapToInt(t -> t.fiducialId).toArray();
                inputs.latestTargetObservation = new TargetObservation(new Rotation2d(result.getBestTarget().getYaw()),
                        new Rotation2d(result.getBestTarget().getPitch()), result.getBestTarget().fiducialId);

                inputs.targets = result.targets.stream().map(t -> new TargetObservation(
                        new Rotation2d(t.getYaw()),
                        new Rotation2d(t.getPitch()),
                        t.fiducialId)).toArray(TargetObservation[]::new);
            } else {
                inputs.targets = new TargetObservation[0];
                inputs.tagIds = new int[0];

            }

        }
        inputs.connected = camera.isConnected();
        inputs.poseObservations = observations.toArray(new PoseObservation[0]);

        inputs.name = camera.getName();
        inputs.robotToCamera = robotToCamera;

        observations.clear();
    }

    public Distance getAvgDistance(PhotonPipelineResult res) {
        double sum = 0;
        for (PhotonTrackedTarget target : res.getTargets()) {
            sum += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        return Meters.of(sum / res.getTargets().size());
    }
}