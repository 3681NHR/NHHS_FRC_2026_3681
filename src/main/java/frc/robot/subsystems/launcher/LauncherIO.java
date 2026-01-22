package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
    
    public default void updateInputs(LauncherIOInputs input){}
    
    public default void setGoal(double goal){}
    public default void setVout(double vout){}
    public default void setOpenLoop(boolean openloop){}

    @AutoLog
    public class LauncherIOInputs{
        public double filteredAngle = 0.0;
        public double filteredSpeed = 0.0;
        public double rawAngle = 0.0;
        public double rawSpeed = 0.0;

        public double motorVoltageOut = 0;
        public double motorCurrentOut = 0;
        public double motorTemp = 0;

        public double goal = 0.0;
        public boolean atSetpoint = false;

        public boolean openLoop = false;

    }
}
