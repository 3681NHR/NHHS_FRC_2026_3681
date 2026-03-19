// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.constants.VisionConstants.CameraConfig;
import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

/** IO implementation for physics sim using PhotonVision simulator. */
public class CameraIOPhotonSim extends CameraIOPhoton {
    private static VisionSystemSim visionSim;

    private final Supplier<Pose2d> poseSupplier;
    private final PhotonCameraSim cameraSim;

    private final LoggedNetworkBoolean simulatePhoton = new LoggedNetworkBoolean("Sim/simulate vision", false);

    /**
     * Creates a new CameraIOPhotonSim.
     *
     * @param poseSupplier Supplier for the robot pose to use in simulation.
     */
    public CameraIOPhotonSim(
            AprilTagFieldLayout layout, CameraConfig config, Supplier<Pose2d> poseSupplier) {
        super(layout, config);
        this.poseSupplier = poseSupplier;

        // Initialize vision sim
        if (visionSim == null) {
            visionSim = new VisionSystemSim("main");
            visionSim.addAprilTags(layout);
        }

        // Add sim camera
        var cameraProperties = new SimCameraProperties();
        cameraProperties.setFPS(45);
        cameraProperties.setAvgLatencyMs(23);
        cameraProperties.setLatencyStdDevMs(20);
        cameraProperties.setCalibration(800, 600, Rotation2d.fromDegrees(110));
        cameraProperties.setCalibError(0.1, 0.02);
        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        visionSim.addCamera(cameraSim, config.robotToCam);
    }

    @Override
    public void updateInputs(CameraIOInputs inputs) {
        if(simulatePhoton.getAsBoolean()){
            visionSim.update(poseSupplier.get());
            super.updateInputs(inputs);
        } else {

            inputs.poseObservations = new PoseObservation[]{new PoseObservation(
                            Microseconds.of(Logger.getTimestamp()).in(Seconds),
                            new Pose3d(poseSupplier.get().getX(), poseSupplier.get().getY(), 0, new Rotation3d(0, 0, poseSupplier.get().getRotation().getRadians())),
                            -1,
                           1,
                            1)};

            inputs.connected = camera.isConnected();

            inputs.name = camera.getName();
            inputs.robotToCamera = super.robotToCamera;
            inputs.connected = true;
        }
    }
}