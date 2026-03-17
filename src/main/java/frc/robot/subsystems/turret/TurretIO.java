package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kelvin;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface TurretIO {

    public default void updateInputs(TurretIOInputs input){}

    public default void setGoal(Angle goal){}
    public default void setVout(Voltage vout){}

    public default void setTolerance(Angle tol){}
    
    @AutoLog
    public class TurretIOInputs {

        public Angle angle = Radians.of(0.0);
        public Angle motorAngle = Radians.of(0.0);
        public AngularVelocity speed = RadiansPerSecond.of(0.0);
        public Angle tolerance = Radians.of(0.0);

        public Voltage motorVoltageOut = Volts.of(0);
        public Current motorCurrentOut = Amps.of(0);
        public Temperature motorTemp = Kelvin.of(0);

        public Angle goal = Radians.of(0.0);
        public Angle setpointPos = Radians.of(0);
        public AngularVelocity setpointVel = RadiansPerSecond.of(0);
        public boolean atSetpoint = false;

        public boolean openLoop = false;

        public Angle angleE1 = Radians.of(0.0);
        public Angle angleE2 = Radians.of(0.0);
    }
}
