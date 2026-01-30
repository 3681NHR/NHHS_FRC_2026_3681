package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.Radians;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.GyroSimulation;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import frc.utils.SparkUtil;

public class GyroIOSim implements GyroIO {

    private final GyroSimulation gyro;

    public GyroIOSim(GyroSimulation gyro) {
        this.gyro = gyro;
    }

    public void reset(double heading) {
        gyro.setRotation(new Rotation2d(heading));
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = true;
        inputs.yawPosition = Radians.of(gyro.getGyroReading().getRadians());
        inputs.yawVelocity = gyro.getMeasuredAngularVelocity();

        inputs.odometryYawPositions = Arrays.stream(gyro.getCachedGyroReadings()).map(e -> Radians.of(e.getRadians())).toArray(e -> new Angle[e]);
        inputs.odometryYawTimestamps = SparkUtil.getSimulationOdometryTimeStamps();

        inputs.angle = new Rotation3d(0, 0, inputs.yawPosition.in(Radians));
    }
}