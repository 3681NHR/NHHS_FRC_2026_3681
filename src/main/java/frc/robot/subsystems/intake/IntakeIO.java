package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    
    public void updateInputs(IntakeIOInputs input);
    
    @AutoLog
    public class IntakeIOInputs{

    }
}
