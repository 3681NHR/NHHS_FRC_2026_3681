package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public interface LauncherIO {
    
    public default void updateInputs(LauncherIOInputs input){}
    
    public default void setGoal(AngularVelocity goal){}
    public default void setVout(Voltage vout){}

    @AutoLog
    public class LauncherIOInputs{
        public Angle angle = Rotations.zero();
        public AngularVelocity speed = RPM.zero();

        public Voltage motorVoltageOut = Volts.zero();
        public Current motorCurrentOut = Amps.zero();
        public Temperature motorTemp = Celsius.zero();

        public AngularVelocity goal = RPM.zero();
        public boolean atSetpoint = false;

        public boolean openLoop = false;

    }
}
