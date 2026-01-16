package frc.robot.subsystems.turret;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;

import static frc.robot.constants.TurretConstants.*;

public class TurretIOSim implements TurretIO {
    
    private double goal = 0.0;
    private boolean openloop = false;
    private double openloopVout = 0.0;

    private final LinearSystemSim sim = new LinearSystemSim<N2, N1, N2>(LinearSystemId.identifyPositionSystem(TURRET_FF.kV(), TURRET_FF.kA()));

    public void updateInputs(TurretIOInputs input){

    }

    public void setGoal(double goal){

    }
    public void setVout(double vout){

    }
    public void setOpenLoop(boolean openloop){

    }
}
