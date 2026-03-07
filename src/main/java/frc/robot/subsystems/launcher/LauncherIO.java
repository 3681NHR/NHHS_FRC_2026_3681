package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public interface LauncherIO {
    
    public default void updateInputs(LauncherIOInputs input){}
    
    public default void setGoal(AngularVelocity goal){}
    public default void setVout(Voltage vout){}

    @AutoLog
    public class LauncherIOInputs{
        public Angle angle;
        public AngularVelocity speed;

        public Voltage motorVoltageOut;
        public Current motorCurrentOut;
        public Temperature motorTemp;

        public AngularVelocity goal;
        public boolean atSetpoint;

        public boolean openLoop;

    }
}
