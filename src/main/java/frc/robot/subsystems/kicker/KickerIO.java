package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Kelvin;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface KickerIO {
    
    public default void updateInputs(KickerIOInputs input) {}

    public default void setVout(Voltage vout) {}
    public default void setGoal(LinearVelocity goal) {}
    
    @AutoLog
    public class KickerIOInputs{
        public LinearVelocity speed = MetersPerSecond.zero();

        public Voltage motorVoltageOut = Volts.zero();
        public Current motorCurrentOut = Amps.zero();
        public Temperature motorTemp = Kelvin.zero().minus(Kelvin.one());

        public boolean hasBall = false;
        public Distance distance = Meters.zero();

        public LinearVelocity goal = MetersPerSecond.zero();

        public boolean openLoop = false;
    }
}
