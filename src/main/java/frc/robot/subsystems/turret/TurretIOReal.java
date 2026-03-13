package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.TurretConstants.*;

import frc.utils.ExtraMath;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.utils.motorWrappers.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class TurretIOReal implements TurretIO {

    private final TalonFX motor = new TalonFX(TURRET_MOTOR_ID);
    private final CANcoder e1 = new CANcoder(TURRET_ENCODER_1_ID);
    private final CANcoder e2 = new CANcoder(TURRET_ENCODER_2_ID);

    private boolean openLoop = false;
    private final MotionMagicVoltage closedLoopControl = new MotionMagicVoltage(Radians.of(0));
    private final VoltageOut openLoopControl = new VoltageOut(0);
    private Angle goal = Radians.of(0);

    private final Alert motorDisconnect = new Alert("Turret motor is disconnected!", AlertType.kError);
    private final Alert e1Disconnect = new Alert("Turret encoder 1 is disconnected!", AlertType.kError);
    private final Alert e2Disconnect = new Alert("Turret encoder 2 is disconnected!", AlertType.kError);

    private boolean homed = false;

    private final LoggedNetworkBoolean resetTurretPos = new LoggedNetworkBoolean("Debug/reset turret pos", false);

    public TurretIOReal(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.MotionMagic = new MotionMagicConfigs()
        .withMotionMagicAcceleration(TURRET_PID_GAINS.maxAccel)
        .withMotionMagicCruiseVelocity(TURRET_PID_GAINS.maxSpeed);

        config.Slot0 = new Slot0Configs()
            .withKP(TURRET_PID_GAINS.kP)
            .withKI(TURRET_PID_GAINS.kI)
            .withKD(TURRET_PID_GAINS.kD)
            .withKS(TURRET_FF_GAINS.kS)
            .withKV(TURRET_FF_GAINS.kV)
            .withKA(TURRET_FF_GAINS.kA);

        config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(TURRET_ANGLE_FORWARD_LIM)
            .withReverseSoftLimitThreshold(TURRET_ANGLE_REVERSE_LIM)
            .withForwardSoftLimitEnable(true)
            .withReverseSoftLimitEnable(true);

        config.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(TURRET_CURRENT_LIM).withSupplyCurrentLimitEnable(true);

        config.MotorOutput = new MotorOutputConfigs()
        .withInverted(TURRET_MOTOR_INVERT ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
        .withNeutralMode(NeutralModeValue.Brake);

        config.Feedback = new FeedbackConfigs().withSensorToMechanismRatio(TURRET_MAIN_GEAR_TEETH/TURRET_MOTOR_GEAR_TEETH);

         config.SoftwareLimitSwitch = new SoftwareLimitSwitchConfigs().withForwardSoftLimitEnable(true)
         .withForwardSoftLimitThreshold(TURRET_ANGLE_FORWARD_LIM)
         .withReverseSoftLimitEnable(true)
         .withReverseSoftLimitThreshold(TURRET_ANGLE_REVERSE_LIM);

        motor.getConfigurator().apply(config);

        TURRET_PID_GAINS.withCallback(() -> {
            motor.getConfigurator().apply(new Slot0Configs()
            .withKP(TURRET_PID_GAINS.kP)
            .withKI(TURRET_PID_GAINS.kI)
            .withKD(TURRET_PID_GAINS.kD));
        });
        TURRET_FF_GAINS.withCallback(() -> {
            motor.getConfigurator().apply(new Slot0Configs()
            .withKS(TURRET_FF_GAINS.kS)
            .withKV(TURRET_FF_GAINS.kV)
            .withKA(TURRET_FF_GAINS.kA));
        });

        if(e1.isConnected() && e2.isConnected()){
            motor.setPosition(getAbsoluteAngle());

            homed = true;
        }
    }

    @Override
    public void updateInputs(TurretIOInputs input){
        
        if(e1.isConnected() && e2.isConnected() && (!homed || resetTurretPos.get())){
            motor.setPosition(getAbsoluteAngle());
            homed = true;
            resetTurretPos.set(false);
        }
        motorDisconnect.set(!motor.isConnected());
        e1Disconnect.set(!e1.isConnected());
        e2Disconnect.set(!e2.isConnected());

        input.angle = getAbsoluteAngle();
        input.motorAngle = motor.getPosition().getValue();
        input.speed = getVelocity();

        input.motorVoltageOut = motor.getMotorVoltage().getValue();
        input.motorCurrentOut = motor.getSupplyCurrent().getValue();
        input.motorTemp = motor.getDeviceTemp().getValue();

        input.goal = goal;
        input.atSetpoint = MathUtil.isNear(goal.in(Radians),motor.getPosition().getValue().in(Radians), TURRET_SETPOINT_TOLERANCE.in(Radians));

        input.setpointPos = Rotations.of(motor.getClosedLoopReference().getValue());
        input.setpointVel = RotationsPerSecond.of(motor.getClosedLoopReferenceSlope().getValue());

        input.openLoop = openLoop;

        input.angleE1 = e1.getAbsolutePosition().getValue();
        input.angleE2 = e2.getAbsolutePosition().getValue();
    }    
    
    @Override
    public void setGoal(Angle goal){
        this.goal = goal;
        openLoop = false;
        motor.setControl(closedLoopControl.withPosition(goal));
    }
    
    @Override
    public void setVout(Voltage vout){
        openLoop = true;
        motor.setControl(openLoopControl.withOutput(vout));
    }

    private Angle getAbsoluteAngle(){

        Angle angle;
        if(e1.isConnected() && e2.isConnected()){
//            angle = Rotations.of(SLOPE * (MathUtil.inputModulus(e2.getAbsolutePosition().getValue().in(Rotations)-e1.getAbsolutePosition().getValue().in(Rotations), 0, 1)));
             angle = Degrees.of(ExtraMath.calculateTurretAngleFromCANCoderDegrees(e1.getAbsolutePosition().getValue().in(Degrees), e2.getAbsolutePosition().getValue().in(Degrees)));

            // motor.setPosition(angle);
        } else {
            if(motor.isConnected() && homed){
                angle = motor.getPosition().getValue();
            } else {
                angle = Rotations.of(0);
            }
        }

        return angle;
    }

    private AngularVelocity getVelocity(){
        int measures = 0;
        double sum = 0;//RPS

        if(e1.isConnected()){
            sum += e1.getVelocity().getValueAsDouble()/(TURRET_MAIN_GEAR_TEETH/TURRET_ENCODER_1_GEAR_TEETH);
            measures++;
        }
        if(e2.isConnected()){
            sum += e2.getVelocity().getValueAsDouble()/(TURRET_MAIN_GEAR_TEETH/TURRET_ENCODER_2_GEAR_TEETH);
            measures++;
        }
        if(motor.isConnected()){
            sum += motor.getVelocity().getValueAsDouble();
            measures++;
        }
        if(measures>0){
            return RotationsPerSecond.of(-sum/measures);//average all velocity readings
        }
        return RotationsPerSecond.of(0);//the bad ending
    }

}
