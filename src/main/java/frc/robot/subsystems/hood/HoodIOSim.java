package frc.robot.subsystems.hood;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.HoodConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
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

    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(HOOD_ID_GAINS.kV/(Math.PI*2), HOOD_ID_GAINS.kA/(Math.PI*2));
    private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<N2, N1, N2>(model, 0.0001, 0.001);

    Angle encoderAngle = Degrees.of(0);
    Angle encoderOffset = Degrees.of(0);

    public HoodIOSim(){
        sim.setState(VecBuilder.fill(Degree.of(35).in(Radians),0));
        encoderOffset = Radians.of(-sim.getOutput().get(0,0));

        pid.setTolerance(HOOD_SETPOINT_TOLERANCE.in(Rotations));
    }
    
    @Override
    public void updateInputs(HoodIOInputs input){
        sim.update(0.02);
        encoderAngle = Radians.of(sim.getOutput().get(0,0)).plus(encoderOffset);

        if(!openloop){
            vout = Volts.of(pid.calculate(encoderAngle.in(Rotations), goal.in(Rotations)));
            vout = vout.plus(Volts.of(ff.calculate(pid.getSetpoint().velocity)));
        }
        
        if(homed){
            //soft limit - cant use internal, as it cant be configured while enabled
            if(encoderAngle.in(Rotations) >= HOOD_MAX_ANGLE.in(Rotations) && vout.in(Volts) > 0){
                vout = Volts.of(0);
            }
            if(encoderAngle.in(Rotations) <= HOOD_MIN_ANGLE.in(Rotations) && vout.in(Volts) < 0){
                vout = Volts.of(0);
            }
        }
        
        if(Radians.of(sim.getOutput().get(0,0)).gt(HOOD_MAX_ANGLE)){
            sim.setState(VecBuilder.fill(HOOD_MAX_ANGLE.in(Radians),0));
            sim.setInput(MathUtil.clamp(vout.in(Volts), -12, 0));

        } else if(Radians.of(sim.getOutput().get(0,0)).lt(HOOD_MIN_ANGLE)){
            sim.setState(VecBuilder.fill(HOOD_MIN_ANGLE.in(Radians),0));
            sim.setInput(MathUtil.clamp(vout.in(Volts), 0, 12));
        } else {
            sim.setInput(vout.in(Volts));
        }
        
        input.angle = encoderAngle;
        input.velocity = RadiansPerSecond.of(sim.getOutput().get(1,0));

        input.atSetpoint = pid.atGoal();

        input.vout = vout;
        input.current = Amps.of(-1);
        input.temp = Kelvin.of(-1);

        input.homed = homed;
        input.openloop = openloop;
        
        input.goal = goal;
        input.setpointPos = Rotations.of(pid.getSetpoint().position);
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
        encoderOffset = encoderAngle.minus(encoderOffset).unaryMinus().plus(pos);
        encoderAngle = Radians.of(sim.getOutput().get(0,0)).plus(encoderOffset);
        pid.reset(encoderAngle.in(Rotations));
    }
    
    public void setHomed(boolean homed){
        this.homed = homed;
    }
}
