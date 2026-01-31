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
    public default void setOpenLoop(boolean openloop){}

    @AutoLog
    public class LauncherIOInputs{
        public Angle filteredAngle;
        public AngularVelocity filteredSpeed;
        public Angle rawAngle;
        public AngularVelocity rawSpeed;

        public Voltage motorVoltageOut;
        public Current motorCurrentOut;
        public Temperature motorTemp;

        public AngularVelocity goal;
        public boolean atSetpoint;

        public boolean openLoop;

    }
}
