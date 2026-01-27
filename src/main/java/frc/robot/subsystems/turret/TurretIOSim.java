package frc.robot.subsystems.turret;

import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;

import static frc.robot.constants.TurretConstants.*;

public class TurretIOSim implements TurretIO {
    
    private double goal = 0.0;
    private boolean openLoop = false;
    private double Vout = 0.0;
    private double angle = 0.0;

    private ProfiledPID pid = new ProfiledPID(TURRET_PID_GAINS);
    private SimpleFF ff = new SimpleFF(TURRET_FF_GAINS);

    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(TURRET_ID_GAINS.kV(), TURRET_ID_GAINS.kA());
    private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<N2, N1, N2>(model, 0.01, 0.1);
    private final KalmanFilter<N2, N1, N2> filter = new KalmanFilter<N2, N1, N2>(Nat.N2(), Nat.N2(), model, VecBuilder.fill(0.2, 1.0), VecBuilder.fill(1.3, 0.7), 0.02);

    public TurretIOSim(){
    }
    public void updateInputs(TurretIOInputs input){
        sim.update(0.02);
        filter.predict(VecBuilder.fill(Vout - Math.min(TURRET_ID_GAINS.kS(), Math.abs(Vout))*Math.signum(sim.getOutput().get(1,0))), 0.02);
        filter.correct(VecBuilder.fill(Vout - Math.min(TURRET_ID_GAINS.kS(), Math.abs(Vout))*Math.signum(sim.getOutput().get(1,0))), sim.getOutput());
        angle = filter.getXhat().get(0,0);

        if(!openLoop){
            Vout = pid.calculate(angle, goal);
            Vout += ff.calculate(pid.getSetpoint().velocity);
        }
        if(DriverStation.isEnabled()){
            sim.setInput(Vout - Math.min(TURRET_ID_GAINS.kS(), Math.abs(Vout))*Math.signum(sim.getOutput().get(1,0)));
        } else {            
            sim.setInput(-Math.min(TURRET_ID_GAINS.kS(), Math.abs(Vout))*Math.signum(sim.getOutput().get(1,0)));
        }
        
        input.filteredAngle = angle;
        input.filteredSpeed = filter.getXhat(1);
        input.rawAngle = sim.getOutput().get(0, 0);
        input.rawSpeed = sim.getOutput().get(1, 0);

        input.motorVoltageOut = Vout;

        input.goal = goal;
        input.setpoint = pid.getSetpoint();
        input.atSetpoint = MathUtil.isNear(goal, angle, TURRET_SETPOINT_TOLERANCE);
    
        input.openLoop = openLoop;
    }

    public void setGoal(double goal){
        this.openLoop = false;
        this.goal = goal;
    }
    public void setVout(double vout){
        this.openLoop = true;
        Vout = vout;
    }
    public void setOpenLoop(boolean openLoop){
        this.openLoop = openLoop;
    }

}
