package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.ClimberConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.constants.Constants;
import frc.utils.controlWrappers.ElevatorFF;
import frc.utils.controlWrappers.ProfiledPID;

public class ClimberIOSim implements ClimberIO {

    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(CLIMBER_ID_GAINS.kV, CLIMBER_ID_GAINS.kA);
    private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<N2, N1, N2>(model, 0.0, 0.001);

    private final ProfiledPID pid = new ProfiledPID(CLIMBER_PID_GAINS);
    private final ElevatorFF ff = new ElevatorFF(CLIMBER_FF_GAINS);

    private boolean openLoop = false;
    private Distance goal = CLIMBER_MIN_POSITION;
    private Voltage appliedVolts = Volts.zero();

    private Distance position = CLIMBER_MIN_POSITION;
    private Distance offset = Meters.zero();

    private boolean homed = CLIMBER_HOME_ON_START;

    @Override
    public void updateInputs(ClimberIOInputs input) {
        sim.update(Constants.EVENT_LOOP_TIME);
        position = Meters.of(sim.getOutput().get(0,0)).plus(offset);

        if (!openLoop && homed) {
            appliedVolts = Volts.of(pid.calculate(position.in(Meters), goal.in(Meters)) + ff.calculate(pid.getSetpoint().velocity));
        }
        
        appliedVolts = Volts.of(MathUtil.clamp(appliedVolts.in(Volts), -RobotController.getBatteryVoltage(),
        RobotController.getBatteryVoltage()));
        
        double voltageInput;
        if (DriverStation.isEnabled()) {
            voltageInput = appliedVolts.in(Volts) - (Math.min(CLIMBER_ID_GAINS.kS, Math.abs(appliedVolts.in(Volts))) * Math.signum(sim.getOutput().get(1, 0)));//TODO add kg
        } else {
            voltageInput  = -CLIMBER_ID_GAINS.kG;
        }
        
        if(Meters.of(sim.getOutput().get(0,0)).gt(CLIMBER_MAX_POSITION)){
            sim.setState(VecBuilder.fill(CLIMBER_MAX_POSITION.in(Meters),0));
            sim.setInput(MathUtil.clamp(voltageInput, -12, 0));

        } else if(Meters.of(sim.getOutput().get(0,0)).lt(CLIMBER_MIN_POSITION)){
            sim.setState(VecBuilder.fill(CLIMBER_MIN_POSITION.in(Meters),0));
            sim.setInput(MathUtil.clamp(voltageInput, 0, 12));
        } else {
            sim.setInput(voltageInput);
        }

        input.position = position;
        input.velocity = MetersPerSecond.of(sim.getOutput().get(1,0));

        input.motorVoltageOut = appliedVolts;

        input.goal = goal;
        input.atSetpoint = pid.atSetpoint();
        input.velocitySetpoint = MetersPerSecond.of(pid.getSetpoint().velocity);
        input.positionSetpoint = Meters.of(pid.getSetpoint().position);

        input.connected = true;
        input.openLoop = openLoop;
        input.homed = homed;
    }

    /** sets the goal position for the climber. */
    @Override
    public void setGoal(Distance goal) {
        openLoop = false;
        this.goal = goal;
    }

    @Override
    public void setVoltage(Voltage voltage) {
        openLoop = true;
        appliedVolts = voltage;
    }

    
    @Override
    public void setHomed(boolean homed){
        this.homed = homed;
    }

    @Override
    public void setPosition(Distance position){
        offset = this.position.minus(offset).unaryMinus().plus(position);
    }
}
