package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.constants.VisionConstants;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightSettings;
import limelight.networktables.PoseEstimate;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import limelight.Limelight;

public class CameraIOLL implements CameraIO {

    protected Limelight camera;
    private LimelightPoseEstimator poseEstimator;
    private String name;

    final Transform3d robotToCamera;

    /**
     * Creates a new CameraIOLL.
     */
    public CameraIOLL(AprilTagFieldLayout layout, VisionConstants.CameraConfig config) {
        this.robotToCamera = config.robotToCam;
        camera = new Limelight(config.name);
        name = config.name;

        camera.getSettings()
                .withCameraOffset(new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation()))
                .withImuMode(LimelightSettings.ImuMode.InternalImuExternalAssist)
                .save();

        this.poseEstimator = camera.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        Optional<PoseEstimate> estimate = poseEstimator.getPoseEstimate();
        if(estimate.isPresent()){
            inputs.tagIds = Arrays.stream(estimate.get().rawFiducials).mapToInt(tag -> tag.id).toArray();
            inputs.poseObservations = new PoseObservation[1];
            inputs.poseObservations[0] = new PoseObservation(
                    estimate.get().timestampSeconds,
                    estimate.get().pose,
                    estimate.get().getAvgTagAmbiguity(),
                    estimate.get().rawFiducials.length,
                    estimate.get().avgTagDist
            );
        } else {
            inputs.tagIds = new int[0];
            inputs.poseObservations = new PoseObservation[0];
        }
//        inputs.connected = camera.getData();

        inputs.name = name;
        inputs.robotToCamera = robotToCamera;

    }

    public double getAvgDistance(PhotonPipelineResult res) {
        double sum = 0;
        for (PhotonTrackedTarget target : res.getTargets()) {
            sum += target.getBestCameraToTarget().getTranslation().getNorm();
        }

        return sum / res.getTargets().size();
    }
}
