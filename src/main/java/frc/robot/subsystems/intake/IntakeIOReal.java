package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.IntakeConstants;
import frc.utils.controlWrappers.ArmFF;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;
import frc.utils.motorWrappers.SparkMax;

public class IntakeIOReal implements IntakeIO {

    // ── Roller ────────────────────────────────────────────────────────────────
    private final SparkMax rollerMotor = new SparkMax(IntakeConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder rollerEncoder = rollerMotor.getEncoder();
    private final SimpleFF rollerFF = new SimpleFF(IntakeConstants.ROLLER_FF_GAINS);
    private final Alert rollerDisconnect = new Alert(
            "Intake roller Spark disconnected, id: " + IntakeConstants.INTAKE_MOTOR_ID, AlertType.kError);
    private boolean rollerOpenLoop = false;
    private AngularVelocity rollerSetpoint = Units.RPM.zero();

    // ── Pivot ─────────────────────────────────────────────────────────────────
    private final SparkMax pivotMotor = new SparkMax(IntakeConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    private final CANcoder pivotEncoder = new CANcoder(IntakeConstants.INTAKE_ENCODER_ID);
    private final ProfiledPID pivotPID = new ProfiledPID(IntakeConstants.PIVOT_PID_GAINS);
    private ArmFF pivotFF = new ArmFF(IntakeConstants.PIVOT_FF_GAINS);
    private final Alert pivotDisconnect = new Alert(
            "Intake pivot Spark disconnected, id: " + IntakeConstants.PIVOT_MOTOR_ID, AlertType.kError);
    private boolean pivotOpenLoop = false;
    private Angle pivotGoal = IntakeConstants.STOWED_ANGLE;

    public IntakeIOReal() {
        // Live-tuning callbacks
        IntakeConstants.ROLLER_FF_GAINS.withCallback(() -> {
            rollerFF.setKs(IntakeConstants.ROLLER_FF_GAINS.kS);
            rollerFF.setKv(IntakeConstants.ROLLER_FF_GAINS.kV);
            rollerFF.setKa(IntakeConstants.ROLLER_FF_GAINS.kA);
        });
        IntakeConstants.PIVOT_PID_GAINS.withCallback(() -> pivotPID.setGains(IntakeConstants.PIVOT_PID_GAINS));
        IntakeConstants.PIVOT_FF_GAINS.withCallback(() -> {
            pivotFF.setKs(IntakeConstants.PIVOT_FF_GAINS.kS);
            pivotFF.setKg(IntakeConstants.PIVOT_FF_GAINS.kG);
            pivotFF.setKv(IntakeConstants.PIVOT_FF_GAINS.kV);
            pivotFF.setKa(IntakeConstants.PIVOT_FF_GAINS.kA);
        });

        pivotPID.setTolerance(IntakeConstants.PIVOT_TOLERANCE.in(Units.Radians));

        // Roller motor config
        SparkMaxConfig rollerCfg = new SparkMaxConfig();
        rollerCfg.idleMode(IdleMode.kBrake)
                 .inverted(IntakeConstants.ROLLER_INVERTED)
                 .smartCurrentLimit(IntakeConstants.ROLLER_SMART_CURRENT_LIMIT);
        rollerMotor.configure(rollerCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Pivot motor config
        SparkMaxConfig pivotCfg = new SparkMaxConfig();
        pivotCfg.idleMode(IdleMode.kBrake)
                .inverted(IntakeConstants.PIVOT_INVERTED)
                .smartCurrentLimit(IntakeConstants.PIVOT_SMART_CURRENT_LIMIT);
        pivotCfg.encoder
                .positionConversionFactor(IntakeConstants.PIVOT_POSITION_CONVERSION_FACTOR)
                .velocityConversionFactor(IntakeConstants.PIVOT_VELOCITY_CONVERSION_FACTOR);
        pivotMotor.configure(pivotCfg, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs input) {
        // ── Roller closed-loop ─────────────────────────────────────────────────
        if (!rollerOpenLoop) {
            double ff = rollerFF.calculate(rollerSetpoint.in(Units.RPM));
            rollerMotor.setVoltage(Units.Volts.of(ff));
        }
        input.rollerConnected = rollerMotor.getLastError() != REVLibError.kCANDisconnected;
        input.rollerVoltageOut = Units.Volts.of(rollerMotor.getAppliedOutput() * rollerMotor.getBusVoltage());
        input.rollerCurrentOut = Units.Amps.of(rollerMotor.getOutputCurrent());
        input.rollerTemp = Units.Celsius.of(rollerMotor.getMotorTemperature());
        input.rollerVelocity = Units.RPM.of(rollerEncoder.getVelocity());
        input.rollerVelocitySetpoint = rollerSetpoint;
        input.rollerOpenLoop = rollerOpenLoop;
        rollerDisconnect.set(!input.rollerConnected);

        // ── Pivot closed-loop ──────────────────────────────────────────────────
        if (!pivotOpenLoop) {
            double pid = pivotPID.calculate(pivotEncoder.getAbsolutePosition().getValue().in(Units.Radians), pivotGoal.in(Units.Radians));
            double ff = pivotFF.calculate(pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity);
            pivotMotor.setVoltage(Units.Volts.of(pid + ff));
        }
        input.pivotConnected = pivotMotor.getLastError() != REVLibError.kCANDisconnected;
        input.pivotVoltageOut = Units.Volts.of(pivotMotor.getAppliedOutput() * pivotMotor.getBusVoltage());
        input.pivotCurrentOut = Units.Amps.of(pivotMotor.getOutputCurrent());
        input.pivotTemp = Units.Celsius.of(pivotMotor.getMotorTemperature());
        input.pivotAngle = pivotEncoder.getAbsolutePosition().getValue();
        input.pivotVelocity = pivotEncoder.getVelocity().getValue();
        input.pivotGoal = pivotGoal;
        input.pivotSetpointPos = Units.Radians.of(pivotPID.getSetpoint().position);
        input.pivotSetpointVel = Units.RadiansPerSecond.of(pivotPID.getSetpoint().velocity);
        input.pivotAtSetpoint = pivotPID.atSetpoint();
        input.pivotOpenLoop = pivotOpenLoop;
        pivotDisconnect.set(!input.pivotConnected);
    }

    @Override
    public void setRollerVelocity(AngularVelocity velocity) {
        rollerOpenLoop = false;
        rollerSetpoint = velocity;
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        rollerOpenLoop = true;
        rollerMotor.setVoltage(voltage);
    }

    @Override
    public void setPivotGoal(Angle goal) {
        pivotOpenLoop = false;
        pivotGoal = goal;
    }

    @Override
    public void setPivotVoltage(Voltage voltage) {
        pivotOpenLoop = true;
        pivotMotor.setVoltage(voltage);
    }

}
