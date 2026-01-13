package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.AutoLog;

public interface LauncherIO {
    
    public void updateInputs(LauncherIOInputs input);
    
    @AutoLog
    public class LauncherIOInputs{

    }
}
