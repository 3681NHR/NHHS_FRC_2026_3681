package frc.robot.subsystems.launcher;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.utils.controlWrappers.PID;
import frc.utils.controlWrappers.SimpleFF;

import static frc.robot.constants.LauncherConstants.*;

public class LauncherIOSim implements LauncherIO {

    double goal = 0.0;
    double vout = 0.0;
    double speed = 0.0;
    boolean openLoop = false;

    private PID pid = new PID(LAUNCHER_PID_GAINS);
    private SimpleFF ff = new SimpleFF(LAUNCHER_FF_GAINS);
    
    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(LAUNCHER_ID_GAINS.kV(), LAUNCHER_ID_GAINS.kA());
    private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<N2, N1, N2>(model, 0.01, 0.1);
    private final KalmanFilter<N2, N1, N2> filter = new KalmanFilter<N2, N1, N2>(Nat.N2(), Nat.N2(), model, VecBuilder.fill(0.2, 1.0), VecBuilder.fill(1.3, 0.7), 0.02);
    
    public void updateInputs(LauncherIOInputs input){
        sim.update(0.02);
        filter.predict(VecBuilder.fill(vout - Math.min(LAUNCHER_ID_GAINS.kS(), Math.abs(vout))*Math.signum(sim.getOutput().get(1,0))), 0.02);
        filter.correct(VecBuilder.fill(vout - Math.min(LAUNCHER_ID_GAINS.kS(), Math.abs(vout))*Math.signum(sim.getOutput().get(1,0))), sim.getOutput());
        speed = filter.getXhat().get(1,0);

        if(!openLoop){
            vout = pid.calculate(speed, goal);
            vout += ff.calculate(goal);
        }
        if(DriverStation.isEnabled()){
            sim.setInput(vout - Math.min(LAUNCHER_ID_GAINS.kS(), Math.abs(vout))*Math.signum(sim.getOutput().get(1,0)));
        } else {
            sim.setInput(0);
        }
        
        input.filteredAngle = filter.getXhat(0);
        input.filteredSpeed = speed;
        input.rawAngle = sim.getOutput().get(0, 0);
        input.rawSpeed = sim.getOutput().get(1, 0);

        input.motorVoltageOut = vout;

        input.goal = goal;
        input.atSetpoint = MathUtil.isNear(goal, speed, LAUNCHER_SETPOINT_TOLERANCE);
    
        input.openLoop = openLoop;
    }

    public void setGoal(double goal){
        this.openLoop = false;
        this.goal = goal;
    }
    public void setvout(double vout){
        this.openLoop = true;
        this.vout = vout;
    }
    public void setOpenLoop(boolean openLoop){
        this.openLoop = openLoop;
    }

}
