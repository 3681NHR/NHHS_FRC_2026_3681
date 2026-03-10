package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
import frc.robot.constants.Constants;
import frc.utils.ExtraMath;
import frc.utils.controlWrappers.ArmFF;
import frc.utils.controlWrappers.ProfiledPID;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.IntakeConstants.*;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.IntakeSimulation.IntakeSide;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;
import org.littletonrobotics.junction.Logger;

import frc.robot.subsystems.SimFuelManager;

public class IntakeIOSim implements IntakeIO {

    //  Roller 
    private Voltage rollerVout = Volts.zero();

    //  Pivot 

    private final ProfiledPID pivotPID = new ProfiledPID(INTAKE_PIVOT_PID_GAINS);
    private ArmFF pivotFF = new ArmFF(INTAKE_PIVOT_FF_GAINS);

    private boolean pivotOpenLoop = false;
    private Angle pivotGoal = INTAKE_STOWED_ANGLE;

    private Voltage pivotVout = Volts.zero();

    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(INTAKE_PIVOT_ID_GAINS.kV, INTAKE_PIVOT_ID_GAINS.kA);
    private final LinearSystemSim<N2, N1, N2> sim = new LinearSystemSim<N2, N1, N2>(model, 0.0, 0.0);

    private IntakeSimulation mapleSimIntake;

    public IntakeIOSim(AbstractDriveTrainSimulation drive) {
        // Live-tuning callbacks
        INTAKE_PIVOT_PID_GAINS.withCallback(() -> pivotPID.setGains(INTAKE_PIVOT_PID_GAINS));
        INTAKE_PIVOT_FF_GAINS.withCallback(() -> {
            pivotFF.setKs(INTAKE_PIVOT_FF_GAINS.kS);
            pivotFF.setKg(INTAKE_PIVOT_FF_GAINS.kG);
            pivotFF.setKv(INTAKE_PIVOT_FF_GAINS.kV);
            pivotFF.setKa(INTAKE_PIVOT_FF_GAINS.kA);
        });

        pivotPID.setTolerance(INTAKE_PIVOT_TOLERANCE.in(Units.Radians));

        mapleSimIntake = IntakeSimulation.OverTheBumperIntake(
            "Fuel",
            drive,
            Inches.of(25),
            Inches.of(6),
            IntakeSide.FRONT,
            10
        );

        SimFuelManager.getInstance().intake = mapleSimIntake;
    }

    @Override
    public void updateInputs(IntakeIOInputs input) {
        sim.update(Constants.EVENT_LOOP_TIME);

        //TODO, kv from recalc, test on bot
        input.rollerVelocity = RPM.of(124.075*rollerVout.in(Volts));

        input.rollerVoltageOut = rollerVout;
        
        input.rollerConnected = true;

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

        //run maple sim intake
        if(input.pivotAngle.lte(Degrees.of(10))
        && input.rollerVoltageOut.gte(Volts.of(3))){
            mapleSimIntake.startIntake();
        } else {
            mapleSimIntake.stopIntake();
        }
        Logger.recordOutput("sim/held fuel", mapleSimIntake.getGamePiecesAmount());
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
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
