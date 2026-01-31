package frc.robot.subsystems.swerve.gyro;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

public interface GyroIO {

    @AutoLog
    public static class GyroIOInputs {
        public boolean connected = false;
        public Angle yawPosition = Radians.of(0);
        public AngularVelocity yawVelocity = RadiansPerSecond.of(0);
        public double[] odometryYawTimestamps = new double[] {};
        public double[] odometryYawPositions = new double[] {};
        public Rotation3d angle = new Rotation3d();
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }

    public default void reset(double headingRad) {
    }
}
