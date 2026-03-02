package frc.robot.subsystems.hood;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.SimpleFF;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.HoodConstants.*;

public class HoodIOReal implements HoodIO {

    private SparkMax motor = new SparkMax(HOOD_MOTOR_ID, MotorType.kBrushless);
    private RelativeEncoder encoder = motor.getEncoder();

    private boolean homed = HOOD_HOME_ON_START;
    private boolean openloop = false;

    private Angle goal = Radians.of(0);
    private Voltage vout = Volts.of(0);

    private ProfiledPID pid = new ProfiledPID(HOOD_PID_GAINS);
    private SimpleFF ff = new SimpleFF(HOOD_FF_GAINS);

    public HoodIOReal(){
        
        var turnConfig = new SparkMaxConfig();
        turnConfig
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit((int) HOOD_CURRENT_LIM.in(Amps))
                .voltageCompensation(12.0);
        turnConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        motor.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    
    @Override
    public void updateInputs(HoodIOInputs input){
        if(openloop){
            vout = Volts.of(pid.calculate(encoder.getPosition(), goal.in(Rotations)));
            vout = vout.plus(Volts.of(ff.calculate(pid.getSetpoint().velocity)));
        }
        motor.setVoltage(vout.in(Volts));
        
        input.angle = Rotations.of(encoder.getPosition());
        input.velocity = RPM.of(encoder.getVelocity());

        input.vout = Volts.of(motor.getAppliedOutput()*motor.getBusVoltage());
        input.current = Amps.of(motor.getOutputCurrent());
        input.temp = Celsius.of(motor.getMotorTemperature());

        input.homed = homed;
        input.openloop = openloop;
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
}
