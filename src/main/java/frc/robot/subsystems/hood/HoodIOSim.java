package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.HoodConstants.*;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;

public class HoodIOSim implements HoodIO {

    private boolean homed = HOOD_HOME_ON_START;
    private boolean openloop = false;

    private Angle goal = Radians.of(0);
    private Voltage vout = Volts.of(0);

    private ProfiledPID pid = new ProfiledPID(HOOD_PID_GAINS);
    private SimpleFF ff = new SimpleFF(HOOD_FF_GAINS);

    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(HOOD_FF_GAINS.kV, HOOD_FF_GAINS.kA);
    private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<N2, N1, N2>(model, 0.01, 0.1);

    private double offset = 0;

    public HoodIOSim(){
        
    }
    
    @Override
    public void updateInputs(HoodIOInputs input){
        sim.update(0.02);

        if(openloop){
            vout = Volts.of(pid.calculate(sim.getOutput().get(0,0), goal.in(Rotations)));
            vout = vout.plus(Volts.of(ff.calculate(pid.getSetpoint().velocity)));
        }
        
        input.angle = Radians.of(sim.getOutput().get(0,0)+offset);
        input.velocity = RadiansPerSecond.of(sim.getOutput().get(0,1));

        input.vout = vout;
        input.current = Amps.of(-1);
        input.temp = Kelvin.of(-1);

        input.homed = homed;
        input.openloop = openloop;
    }
    
    @Override
    public void setGoal(Angle goal){
        openloop = false;
        this.goal = goal;
    }
    @Override
    public void setVout(Voltage vout){
        openloop = true;
        this.vout = vout;
    }

    public void setPos(Angle pos){
        offset = -(sim.getOutput().get(0,0)+offset) + pos.in(Radians);
    }
    
    public void setHomed(boolean homed){
        this.homed = homed;
    }
}
