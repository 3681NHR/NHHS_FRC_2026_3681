package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.constants.Constants;
import frc.utils.ExtraMath;
import frc.utils.controlWrappers.ArmFF;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.*;

public class IntakeIOSim implements IntakeIO {

    //  Roller 

    private final SimpleFF rollerFF = new SimpleFF(INTAKE_ROLLER_FF_GAINS);

    private boolean rollerOpenLoop = false;
    private AngularVelocity rollerGoal = Units.RPM.zero();

    private Voltage rollerVout = Volts.zero();

    //  Pivot 

    private final ProfiledPID pivotPID = new ProfiledPID(INTAKE_PIVOT_PID_GAINS);
    private ArmFF pivotFF = new ArmFF(INTAKE_PIVOT_FF_GAINS);

    private boolean pivotOpenLoop = false;
    private Angle pivotGoal = INTAKE_STOWED_ANGLE;

    private Voltage pivotVout = Volts.zero();

    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(INTAKE_PIVOT_ID_GAINS.kV, INTAKE_PIVOT_ID_GAINS.kA);
    private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<N2, N1, N2>(model, 0.0, 0.0);

    public IntakeIOSim() {
        // Live-tuning callbacks
        INTAKE_ROLLER_FF_GAINS.withCallback(() -> {
            rollerFF.setKs(INTAKE_ROLLER_FF_GAINS.kS);
            rollerFF.setKv(INTAKE_ROLLER_FF_GAINS.kV);
            rollerFF.setKa(INTAKE_ROLLER_FF_GAINS.kA);
        });
        INTAKE_PIVOT_PID_GAINS.withCallback(() -> pivotPID.setGains(INTAKE_PIVOT_PID_GAINS));
        INTAKE_PIVOT_FF_GAINS.withCallback(() -> {
            pivotFF.setKs(INTAKE_PIVOT_FF_GAINS.kS);
            pivotFF.setKg(INTAKE_PIVOT_FF_GAINS.kG);
            pivotFF.setKv(INTAKE_PIVOT_FF_GAINS.kV);
            pivotFF.setKa(INTAKE_PIVOT_FF_GAINS.kA);
        });

        pivotPID.setTolerance(INTAKE_PIVOT_TOLERANCE.in(Units.Radians));

    }

    @Override
    public void updateInputs(IntakeIOInputs input) {
        sim.update(Constants.EVENT_LOOP_TIME);
        //  Roller closed-loop 
        if (!rollerOpenLoop) {
            rollerVout = Volts.of(rollerFF.calculate(rollerGoal.in(Units.RPM)));
        }

        input.rollerVelocity = rollerGoal;
        input.rollerGoal = rollerGoal;

        input.rollerVoltageOut = rollerVout;
        
        input.rollerConnected = true;
        input.rollerOpenLoop = rollerOpenLoop;

        //  Pivot closed-loop 
        if (!pivotOpenLoop) {
            pivotVout = Volts.of(pivotPID.calculate(sim.getOutput().get(0,0), pivotGoal.in(Units.Radians)));
            pivotVout = pivotVout.plus(Volts.of(pivotFF.calculate(sim.getOutput().get(0,0), pivotPID.getSetpoint().velocity)));
        }
        pivotVout = Volts.of(pivotVout.in(Volts) 
        - ExtraMath.lesser(INTAKE_PIVOT_ID_GAINS.kS*Math.signum(sim.getOutput().get(1,0)), pivotVout.in(Volts))
        - INTAKE_PIVOT_ID_GAINS.kG*Math.cos(sim.getOutput().get(0,0))
        );
        if(Radians.of(sim.getOutput().get(0,0)).gt(INTAKE_PIVOT_MAX_ANGLE)){
            sim.setState(VecBuilder.fill(INTAKE_PIVOT_MAX_ANGLE.in(Radians),0));
            sim.setInput(MathUtil.clamp(pivotVout.in(Volts), -12, 0));

        } else if(Radians.of(sim.getOutput().get(0,0)).lt(INTAKE_PIVOT_MIN_ANGLE)){
            sim.setState(VecBuilder.fill(INTAKE_PIVOT_MIN_ANGLE.in(Radians),0));
            sim.setInput(MathUtil.clamp(pivotVout.in(Volts), 0, 12));
        } else {
            sim.setInput(pivotVout.in(Volts));
        }

        input.pivotAngle = Radians.of(sim.getOutput().get(0,0));
        input.pivotVelocity = RadiansPerSecond.of(sim.getOutput().get(1,0));

        input.pivotGoal = pivotGoal;
        input.pivotSetpointPos = Units.Radians.of(pivotPID.getSetpoint().position);
        input.pivotSetpointVel = Units.RadiansPerSecond.of(pivotPID.getSetpoint().velocity);
        input.pivotAtSetpoint = pivotPID.atSetpoint();

        input.pivotVoltageOut = pivotVout;
        
        input.pivotMotorConnected = true;
        input.pivotEncoderConnected = true;
        input.pivotOpenLoop = pivotOpenLoop;
    }

    @Override
    public void setRollerVelocity(AngularVelocity velocity) {
        rollerOpenLoop = false;
        rollerGoal = velocity;
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        rollerOpenLoop = true;
        rollerVout = voltage;
    }

    @Override
    public void setPivotGoal(Angle goal) {
        pivotOpenLoop = false;
        pivotGoal = goal;
    }

    @Override
    public void setPivotVoltage(Voltage voltage) {
        pivotOpenLoop = true;
        pivotVout = voltage;
    }
}
