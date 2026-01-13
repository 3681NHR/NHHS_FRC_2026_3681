package frc.robot.subsystems.turret;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    
    TurretIO io;
    TurretIOInputsAutoLogged in = new TurretIOInputsAutoLogged();

    public Turret(){

    }

    @Override
    public void periodic(){

    }
}
