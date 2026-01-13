package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {

    public void updateInputs(TurretIOInputs input);
    
    @AutoLog
    public class TurretIOInputs{

    }
}
