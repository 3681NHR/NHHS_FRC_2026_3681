package frc.robot.subsystems.fuelVision;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.fuelVision.FuelVisionIO.FuelVisionIOInputs;

public class FuelVision extends SubsystemBase {
    private FuelVisionIO io;
    private FuelVisionIOInputsAutoLogged inputs = new FuelVisionIOInputsAutoLogged();

    public FuelVision(FuelVisionIO io){
        this.io = io;
    }


    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("IO/FuelVision", inputs);
    }
}
