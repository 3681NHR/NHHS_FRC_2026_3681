package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants.CameraConfig;

public class CameraIOPhoton implements CameraIO {

    protected final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;

    final Transform3d robotToCamera;

    /**
     * Creates a new CameraIOPhoton.
     */
    public CameraIOPhoton(AprilTagFieldLayout layout, CameraConfig config) {
        camera = new PhotonCamera(config.name);
        this.robotToCamera = config.robotToCam;

        this.poseEstimator = new PhotonPoseEstimator(layout, robotToCamera);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        ArrayList<PoseObservation> observations = new ArrayList<>();

        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            Optional<EstimatedRobotPose> estimate = poseEstimator.estimateCoprocMultiTagPose(result);
            if (!estimate.isPresent()) {
                estimate = poseEstimator.estimateLowestAmbiguityPose(result);
            }
            if (estimate.isPresent()) {
                observations.add(new PoseObservation(
                        estimate.get().timestampSeconds,
                        estimate.get().estimatedPose,
                        result.multitagResult.map(multiTargetPNPResult -> multiTargetPNPResult.estimatedPose.ambiguity).orElse(-1.0),
                        estimate.get().targetsUsed.size(),
                        getAvgDistance(result),
                        PoseObservationType.PHOTONVISION));
                inputs.tagIds = estimate.get().targetsUsed.stream().mapToInt(t -> t.fiducialId).toArray();
                inputs.latestTargetObservation = new TargetObservation(new Rotation2d(result.getBestTarget().getYaw()),
                        new Rotation2d(result.getBestTarget().getPitch()), result.getBestTarget().fiducialId);

            } else {
                inputs.tagIds = new int[0];

            }

        }
        inputs.connected = camera.isConnected();
        inputs.poseObservations = observations.toArray(new PoseObservation[0]);

        inputs.name = camera.getName();
        inputs.robotToCamera = robotToCamera;

        observations.clear();
    }

    public double getAvgDistance(PhotonPipelineResult res) {
        double sum = 0;
        for (PhotonTrackedTarget target : res.getTargets()) {
            sum += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        return sum / res.getTargets().size();
    }
}