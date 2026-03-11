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
    
    public default void updateInputs(HoodIOInputs input){}

    public default void setGoal(Angle goal){}
    public default void setVout(Voltage vout){}
    public default void setPos(Angle pos){}
    public default void setHomed(boolean homed){}

    public default void reset(){}
    
    @AutoLog
    public class HoodIOInputs{
        public Angle angle = Radians.of(0);
        public Angle goal = Radians.of(0);
        public Angle setpointPos = Radians.of(0);
        public AngularVelocity velocity = RadiansPerSecond.of(00);
        public boolean homed = false;
        public boolean atSetpoint = false;
        
        public Voltage vout = Volts.of(0);
        public Current current = Amps.of(0);
        public Temperature temp = Celsius.of(0);

        public boolean openloop = false;
    }
}
