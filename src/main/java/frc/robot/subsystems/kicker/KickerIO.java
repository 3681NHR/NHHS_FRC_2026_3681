package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kelvin;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface KickerIO {
    
    public default void updateInputs(KickerIOInputs input) {}

    public default void setVout(Voltage vout) {}
    public default void setGoal(AngularVelocity goal) {}
    public default void setOpenLoop(boolean openLoop) {}
    
    @AutoLog
    public class KickerIOInputs{
        public AngularVelocity speed = RadiansPerSecond.of(0);

        public Voltage motorVoltageOut = Volts.of(0);
        public Current motorCurrentOut = Amps.of(0);
        public Temperature motorTemp = Kelvin.of(-1);

        public boolean hasBall = false;
        public Distance distance = Meters.of(0.0);

        public AngularVelocity goal = RadiansPerSecond.of(0);

        public boolean openLoop = false;
    }
}
