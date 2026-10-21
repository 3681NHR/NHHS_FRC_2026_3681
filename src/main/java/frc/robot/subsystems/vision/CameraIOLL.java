package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.swerve.Drive;
import limelight.networktables.*;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

import limelight.Limelight;

import static edu.wpi.first.units.Units.DegreesPerSecond;

public class CameraIOLL implements CameraIO {

    protected Limelight camera;
    private LimelightPoseEstimator poseEstimator;
    private String name;
    private Drive drive;
    private LimelightSettings.ImuMode imuMode = LimelightSettings.ImuMode.ExternalImu;

    final Transform3d robotToCamera;


    /**
     * Creates a new CameraIOLL.
     */
    public CameraIOLL(AprilTagFieldLayout layout, VisionConstants.CameraConfig config, Drive drive) {
        this.robotToCamera = config.robotToCam;
        camera = new Limelight(config.name);
        name = config.name;
        this.drive = drive;

        camera.getSettings()
                .withCameraOffset(new Pose3d(robotToCamera.getTranslation(), robotToCamera.getRotation()))
                .withImuMode(LimelightSettings.ImuMode.InternalImuExternalAssist)
                .withImuAssistAlpha(0.01)
                .save();

        this.poseEstimator = camera.createPoseEstimator(LimelightPoseEstimator.EstimationMode.MEGATAG2);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        if(DriverStation.isDisabled()){
            imuMode = LimelightSettings.ImuMode.ExternalImu;//seeding imu
        } else {
            imuMode = LimelightSettings.ImuMode.InternalImuExternalAssist;
        }
        Logger.recordOutput("Limelight/"+name+"/IMU mode", imuMode);
        camera.getSettings()
                .withImuMode(imuMode)
                .withRobotOrientation(new Orientation3d(
                        new Rotation3d(0,0,drive.getRotation().getRadians()),
                        drive.getAngulerVelocity(),
                        DegreesPerSecond.of(0),
                        DegreesPerSecond.of(0)))
                .save();


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
