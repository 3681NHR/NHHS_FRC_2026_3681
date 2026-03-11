package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.utils.motorWrappers.SparkMax;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.utils.controlWrappers.ProfiledPID;
import frc.utils.controlWrappers.ElevatorFF;

import static frc.robot.constants.ClimberConstants.*;

public class ClimberIOReal implements ClimberIO {
    private final SparkMax motor = new SparkMax(CLIMBER_MOTOR_ID, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();

    private final ProfiledPID pid = new ProfiledPID(CLIMBER_PID_GAINS);
    private final ElevatorFF ff = new ElevatorFF(CLIMBER_FF_GAINS);

    private boolean openLoop = false;
    private Alert disconnect = new Alert("Climber Spark is disconnected!", AlertType.kError);
    
    private Distance goal = CLIMBER_MIN_POSITION;

    private boolean homed = CLIMBER_HOME_ON_START;
    
    public ClimberIOReal() {
        CLIMBER_PID_GAINS.withCallback(() -> {
            pid.setGains(CLIMBER_PID_GAINS);
        });
        CLIMBER_FF_GAINS.withCallback(() -> {
            ff.setKs(CLIMBER_FF_GAINS.kS);
            ff.setKv(CLIMBER_FF_GAINS.kV);
            ff.setKa(CLIMBER_FF_GAINS.kA);
            ff.setKg(CLIMBER_FF_GAINS.kG);
        });
        
        
        SparkMaxConfig motorConfig = new SparkMaxConfig();

        motorConfig.idleMode(IdleMode.kBrake).inverted(CLIMBER_INVERTED);
        motorConfig.encoder
            .positionConversionFactor(CLIMBER_POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(CLIMBER_VELOCITY_CONVERSION_FACTOR);

        motor.configure(motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pid.setTolerance(CLIMBER_SETPOINT_TOLERANCE.in(Meters));
    }
    
    @Override
    public void updateInputs(ClimberIOInputs input){
        if (!openLoop && homed) {
            motor.setVoltage(Units.Volts.of(pid.calculate(encoder.getPosition(), goal.in(Meters)) + ff.calculate(pid.getSetpoint().velocity)));
        }
        input.position = Units.Meters.of(encoder.getPosition());
        input.velocity = Units.MetersPerSecond.of(encoder.getVelocity());

        input.motorVoltageOut = Units.Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
        input.motorCurrentOut = Units.Amps.of(motor.getOutputCurrent());
        input.motorTemp = Units.Celsius.of(motor.getMotorTemperature());
        
        input.goal = goal;
        input.positionSetpoint = Units.Meters.of(pid.getSetpoint().position);
        input.velocitySetpoint = Units.MetersPerSecond.of(pid.getSetpoint().velocity);
        input.atSetpoint = pid.atSetpoint();
        
        input.connected = motor.getLastError() != REVLibError.kCANDisconnected;
        input.openLoop = openLoop;
        input.homed = homed;

        disconnect.set(!input.connected);
    }

    @Override
    public void setVoltage(Voltage voltage){
        openLoop = true;
        motor.setVoltage(voltage);
    }

    @Override
    public void setGoal(Distance goal){
        openLoop = false;
        this.goal = goal;
    }

    @Override
    public void setHomed(boolean homed){
        this.homed = homed;
    }

    @Override
    public void setPosition(Distance position){
        encoder.setPosition(position.in(Meters));
    }
}

