package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    IntakeIO io;
    IntakeIOInputsAutoLogged in = new IntakeIOInputsAutoLogged();

    public Intake(){

    }

    @Override
    public void periodic(){

    }
}
