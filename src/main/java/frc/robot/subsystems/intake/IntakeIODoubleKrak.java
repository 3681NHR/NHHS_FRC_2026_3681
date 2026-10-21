package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.utils.controlWrappers.ArmFF;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.motorWrappers.TalonFX;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.IntakeConstants.*;

public class IntakeIODoubleKrak implements IntakeIO {

    private final TalonFX rollerMotor = new TalonFX(INTAKE_ROLLER_MOTOR_ID);
    private final StatusSignal<Voltage> rollerMotorVoltage = rollerMotor.getMotorVoltage();
    private final StatusSignal<Temperature> rollerMotorTemp = rollerMotor.getDeviceTemp();
    private final StatusSignal<Current> rollerMotorCurrent = rollerMotor.getSupplyCurrent();
    private final StatusSignal<AngularVelocity> rollerMotorSpeed = rollerMotor.getVelocity();

    private final VoltageOut rollerMotorVoltageControl = new VoltageOut(0);
    private final Alert rollerMotorDisconnect = new Alert("Intake roller disconnected!", AlertType.kError);

    private final TalonFX pivotMotor = new TalonFX(INTAKE_PIVOT_MOTOR_ID);
    private final StatusSignal<Voltage> pivotMotorVoltage = pivotMotor.getMotorVoltage();
    private final StatusSignal<Temperature> pivotMotorTemp = pivotMotor.getDeviceTemp();
    private final StatusSignal<Current> pivotMotorCurrent = pivotMotor.getSupplyCurrent();

    private final VoltageOut pivotMotorVoltageControl = new VoltageOut(0);
    private final CANcoder pivotEncoder = new CANcoder(INTAKE_PIVOT_ENCODER_ID);

    private final ProfiledPID pivotPID = new ProfiledPID(INTAKE_PIVOT_PID_GAINS);
    private ArmFF pivotFF = new ArmFF(INTAKE_PIVOT_FF_GAINS);

    private final Alert pivotMotorDisconnect = new Alert("Intake pivot Kraken disconnected!", AlertType.kError);
    private final Alert pivotEncoderDisconnect = new Alert("Intake pivot encoder disconnected!", AlertType.kError);

    private boolean pivotOpenLoop = false;
    private Angle pivotGoal = INTAKE_STOWED_ANGLE;

    public IntakeIODoubleKrak() {
        INTAKE_PIVOT_PID_GAINS.withCallback(() -> pivotPID.setGains(INTAKE_PIVOT_PID_GAINS));
        INTAKE_PIVOT_FF_GAINS.withCallback(() -> {
            pivotFF.setKs(INTAKE_PIVOT_FF_GAINS.kS);
            pivotFF.setKg(INTAKE_PIVOT_FF_GAINS.kG);
            pivotFF.setKv(INTAKE_PIVOT_FF_GAINS.kV);
            pivotFF.setKa(INTAKE_PIVOT_FF_GAINS.kA);
        });

        pivotPID.setTolerance(INTAKE_PIVOT_TOLERANCE.in(Units.Radians));

        TalonFXConfiguration rollerCfg = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(INTAKE_ROLLER_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(INTAKE_ROLLER_CURRENT_LIM));
        rollerMotor.getConfigurator().apply(rollerCfg);

        TalonFXConfiguration pivotCfg = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(INTAKE_PIVOT_INVERTED ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withSupplyCurrentLimit(INTAKE_PIVOT_CURRENT_LIM))
                .withFeedback(new FeedbackConfigs()
                        .withRotorToSensorRatio(25.0)
                        .withSensorToMechanismRatio(1.0));
        pivotMotor.getConfigurator().apply(pivotCfg);
    }

    @Override
    public void updateInputs(IntakeIOInputs input) {
        input.rollerVelocity = rollerMotorSpeed.getValue();

        input.rollerVoltageOut = rollerMotorVoltage.getValue();
        input.rollerCurrentOut = rollerMotorCurrent.getValue();
        input.rollerTemp = rollerMotorTemp.getValue();

        input.rollerConnected = rollerMotor.isConnected();
        rollerMotorDisconnect.set(!input.rollerConnected);

        if (!pivotOpenLoop) {
            double pid = pivotPID.calculate(pivotEncoder.getAbsolutePosition().getValue().in(Units.Radians), pivotGoal.in(Units.Radians));
            double ff = pivotFF.calculate(pivotEncoder.getAbsolutePosition().getValue().in(Radians), pivotPID.getSetpoint().velocity);
            pivotMotor.setControl(pivotMotorVoltageControl.withOutput(Volts.of(pid + ff)));
        }
        input.pivotAngle = pivotEncoder.getAbsolutePosition().getValue();
        input.pivotVelocity = pivotEncoder.getVelocity().getValue();

        input.pivotGoal = pivotGoal;
        input.pivotSetpointPos = Units.Radians.of(pivotPID.getSetpoint().position);
        input.pivotSetpointVel = Units.RadiansPerSecond.of(pivotPID.getSetpoint().velocity);
        input.pivotAtSetpoint = pivotPID.atSetpoint();

        input.pivotVoltageOut = pivotMotorVoltage.getValue();
        input.pivotCurrentOut = pivotMotorCurrent.getValue();
        input.pivotTemp = pivotMotorTemp.getValue();

        input.pivotMotorConnected = pivotMotor.isConnected();
        input.pivotEncoderConnected = pivotEncoder.isConnected();
        input.pivotOpenLoop = pivotOpenLoop;

        pivotMotorDisconnect.set(!input.pivotMotorConnected);
        pivotEncoderDisconnect.set(!input.pivotEncoderConnected);
    }

    @Override
    public void setRollerVoltage(Voltage voltage) {
        rollerMotor.setControl(rollerMotorVoltageControl.withOutput(voltage));
    }

    @Override
    public void setPivotGoal(Angle goal) {
        pivotOpenLoop = false;
        pivotGoal = goal;
    }

    @Override
    public void setPivotVoltage(Voltage voltage) {
        pivotOpenLoop = true;
        pivotMotor.setControl(pivotMotorVoltageControl.withOutput(voltage));
    }
}
