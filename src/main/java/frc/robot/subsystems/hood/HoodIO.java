package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface HoodIO {
    
    public void updateInputs(HoodIOInputs input);

    public void setGoal(Angle goal);
    public void setVout(Voltage vout);
    
    @AutoLog
    public class HoodIOInputs{
        public Angle angle = Radians.of(0);
        public AngularVelocity velocity = RadiansPerSecond.of(00);
        public boolean homed = false;
        
        public Voltage vout = Volts.of(0);
        public Current current = Amps.of(0);
        public Temperature temp = Celsius.of(0);

        public boolean openloop = false;
    }
}
