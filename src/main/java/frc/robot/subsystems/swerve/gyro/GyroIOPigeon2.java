package frc.robot.subsystems.swerve.gyro;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import frc.utils.SparkOdometryThread;

import static edu.wpi.first.units.Units.Radian;
import static frc.robot.constants.DriveConstants.*;

import java.util.Queue;

/** IO implementation for Pigeon 2. */
public class GyroIOPigeon2 implements GyroIO {
    private final Pigeon2 pigeon = new Pigeon2(GYRO_ID);
    private final StatusSignal<Angle> yaw = pigeon.getYaw();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Time> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = pigeon.getAngularVelocityZWorld();

    public GyroIOPigeon2() {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(ODOMETRY_FREQ);
        yawVelocity.setUpdateFrequency(50.0);
        pigeon.optimizeBusUtilization();
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = yaw.getValue();
        inputs.yawVelocity = yawVelocity.getValue();

        inputs.odometryYawTimestamps = yawTimestampQueue.stream().toArray(e -> new Time[e]);
        inputs.odometryYawPositions = yawPositionQueue.stream().toArray(e -> new Angle[e]);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();

        inputs.angle = new Rotation3d(
                pigeon.getRoll().getValue().in(Radian),
                pigeon.getPitch().getValue().in(Radian),
                pigeon.getYaw().getValue().in(Radian));

    }

    public void reset(double heading) {
        pigeon.setYaw(Units.radiansToDegrees(heading));
    }
}