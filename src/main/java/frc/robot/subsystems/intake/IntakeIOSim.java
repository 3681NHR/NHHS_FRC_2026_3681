package frc.robot.subsystems.intake;

public class IntakeIOSim implements IntakeIO {
    
    public void updateInputs(IntakeIOInputs input){
        
    }
    public void setVout(double volt){
        throw new UnsupportedOperationException("Cannot set voltage in simulation");
    }
}
