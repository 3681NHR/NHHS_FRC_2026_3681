package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    IntakeIO io;
    IntakeIOInputsAutoLogged in = new IntakeIOInputsAutoLogged();

    public Intake(IntakeIO io) {
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(in);
        Logger.processInputs("IO/Intake", in);
    }
    public Command voltageControl(double volt){
        return run(() -> {
            io.setVout(volt);
        }).withName("Intake Voltage Control");
    }
}
