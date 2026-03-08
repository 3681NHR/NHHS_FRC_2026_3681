package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.constants.Constants;
import frc.robot.constants.IntakeConstants;
import frc.utils.controlWrappers.ArmFF;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;

public class IntakeIOSim implements IntakeIO {

    //  Roller 
    private final SimpleFF rollerFF = new SimpleFF(IntakeConstants.ROLLER_FF_GAINS);
    private boolean rollerOpenLoop = false;
    private AngularVelocity rollerSetpoint = Units.RPM.zero();
    private AngularVelocity rollerVelocity = Units.RPM.zero();
    private double rollerAppliedVolts = 0.0;

    //  Pivot 
    private final ProfiledPID pivotPID = new ProfiledPID(IntakeConstants.PIVOT_PID_GAINS);
    private final ArmFF pivotFF = new ArmFF(IntakeConstants.PIVOT_FF_GAINS);
    private boolean pivotOpenLoop = false;
    private Angle pivotGoal = IntakeConstants.STOWED_ANGLE;
    private Angle pivotAngle = IntakeConstants.STOWED_ANGLE;
    private AngularVelocity pivotVelocity = Units.RadiansPerSecond.zero();
    private double pivotAppliedVolts = 0.0;

    public IntakeIOSim() {
        pivotPID.setTolerance(IntakeConstants.PIVOT_TOLERANCE.in(Units.Radians));
    }

    @Override
    public void updateInputs(IntakeIOInputs input) {
        double battery = RobotController.getBatteryVoltage();
        double dt = Constants.EVENT_LOOP_TIME;

        //  Roller 
        if (!rollerOpenLoop) {
            double ff = rollerFF.calculate(rollerSetpoint.in(Units.RPM));
            rollerAppliedVolts = MathUtil.clamp(ff, -battery, battery);
        }
        // First-order lag: time constant ~0.05 s; NEO free-speed ~5880 RPM @ 12 V
        double targetRPM = (rollerAppliedVolts / 12.0) * 5880.0;
        rollerVelocity = Units.RPM.of(rollerVelocity.in(Units.RPM) + (targetRPM - rollerVelocity.in(Units.RPM)) * dt / 0.05);

        input.rollerConnected = true;
        input.rollerVoltageOut = Units.Volts.of(rollerAppliedVolts);
        input.rollerCurrentOut = Units.Amps.zero();
        input.rollerTemp = Units.Celsius.zero();
        input.rollerVelocity = rollerVelocity;
        input.rollerVelocitySetpoint = rollerSetpoint;
        input.rollerOpenLoop = rollerOpenLoop;

        //  Pivot 
        if (!pivotOpenLoop) {
            double pid = pivotPID.calculate(pivotAngle.in(Units.Radians), pivotGoal.in(Units.Radians));
            double ff = pivotFF.calculate(pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity);
            pivotAppliedVolts = MathUtil.clamp(pid + ff, -battery, battery);
        }
        // Simple Euler integration: kVeff = 5.0 rad/s/V
        final double kVeff = 5.0;
        pivotVelocity = Units.RadiansPerSecond.of(pivotAppliedVolts * kVeff);
        pivotAngle = pivotAngle.plus(Units.Radians.of(pivotVelocity.in(Units.RadiansPerSecond) * dt));

        input.pivotConnected = true;
        input.pivotVoltageOut = Units.Volts.of(pivotAppliedVolts);
        input.pivotCurrentOut = Units.Amps.zero();
        input.pivotTemp = Units.Celsius.zero();
        input.pivotAngle = pivotAngle;
        input.pivotVelocity = pivotVelocity;
        input.pivotGoal = pivotGoal;
        input.pivotSetpointPos = Units.Radians.of(pivotPID.getSetpoint().position);
        input.pivotSetpointVel = Units.RadiansPerSecond.of(pivotPID.getSetpoint().velocity);
        input.pivotAtSetpoint = pivotPID.atSetpoint();
        input.pivotOpenLoop = pivotOpenLoop;
    }

    @Override
    public void setRollerVelocity(AngularVelocity velocity) {
        rollerOpenLoop = false;
        rollerSetpoint = velocity;
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        rollerOpenLoop = true;
        rollerAppliedVolts = MathUtil.clamp(voltage.in(Units.Volts),
                -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    }

    @Override
    public void setPivotGoal(Angle goal) {
        pivotOpenLoop = false;
        pivotGoal = goal;
    }

    @Override
    public void setPivotVoltage(Voltage voltage) {
        pivotOpenLoop = true;
        pivotAppliedVolts = MathUtil.clamp(voltage.in(Units.Volts),
                -RobotController.getBatteryVoltage(), RobotController.getBatteryVoltage());
    }
}
