package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.LauncherConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.Alert;
import frc.utils.Alert.AlertType;

public class LauncherIOReal implements LauncherIO {

    TalonFX motor = new TalonFX(LAUNCHER_MOTOR_ID);

    VoltageOut openLoopRequest = new VoltageOut(0);
    VelocityVoltage closedLoopRequest = new VelocityVoltage(0);
    
    double goal = 0.0;
    double vout = 0.0;
    double speed = 0.0;
    boolean openLoop = false;

    Alert overheat = new Alert("", AlertType.kError);//dynamic text, dont set here
    Alert disconnect = new Alert("Launcher motor disconnected!", AlertType.kError);
    
    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(LAUNCHER_ID_GAINS.kV(), LAUNCHER_ID_GAINS.kA());
    private final KalmanFilter<N2, N1, N2> filter = new KalmanFilter<N2, N1, N2>(Nat.N2(), Nat.N2(), model, VecBuilder.fill(0.2, 1.0), VecBuilder.fill(1.3, 0.7), 0.02);
    
    public LauncherIOReal(){
        motor.getConfigurator()
        .apply(new Slot0Configs()
            .withKP(LAUNCHER_PID_GAINS.kP())
            .withKI(LAUNCHER_PID_GAINS.kI())
            .withKD(LAUNCHER_PID_GAINS.kD())
            .withKS(LAUNCHER_FF_GAINS.kS())
            .withKV(LAUNCHER_FF_GAINS.kV())
            .withKA(LAUNCHER_FF_GAINS.kA())
        );
    }

    public void updateInputs(LauncherIOInputs input){
        filter.predict(VecBuilder.fill(vout - Math.min(LAUNCHER_ID_GAINS.kS(), Math.abs(vout))*Math.signum(motor.getVelocity().getValueAsDouble())), 0.02);
        filter.correct(VecBuilder.fill(vout - Math.min(LAUNCHER_ID_GAINS.kS(), Math.abs(vout))*Math.signum(motor.getVelocity().getValueAsDouble())), VecBuilder.fill(motor.getPosition().getValueAsDouble(), motor.getVelocity().getValueAsDouble()));
        speed = filter.getXhat().get(1,0);

        if(DriverStation.isEnabled()){
            if(!openLoop){
                motor.setControl(closedLoopRequest.withVelocity(RadiansPerSecond.of(goal)));
            } else{
                motor.setControl(openLoopRequest.withOutput(Volts.of(vout)));
            }
        } else {
            motor.stopMotor();
        }
        if(motor.getDeviceTemp().getValueAsDouble() > LAUNCHER_MAX_TEMP){
            overheat.set(true);
            overheat.setText("Launcher motor overheat! ("+motor.getDeviceTemp().getValueAsDouble()+"C)");
        } else {
            overheat.set(false);
        }
        disconnect.set(motor.isConnected());
        
        input.filteredAngle = filter.getXhat(0);
        input.filteredSpeed = speed;
        input.rawAngle = motor.getPosition().getValueAsDouble();
        input.rawSpeed = motor.getVelocity().getValueAsDouble();

        input.motorCurrentOut = motor.getStatorCurrent().getValueAsDouble();
        input.motorTemp = motor.getDeviceTemp().getValueAsDouble();
        input.motorVoltageOut = vout;

        input.goal = goal;
        input.atSetpoint = MathUtil.isNear(goal, speed, LAUNCHER_SETPOINT_TOLERANCE);
    
        input.openLoop = openLoop;
    }

    public void setGoal(double goal){
        this.openLoop = false;
        this.goal = goal;
    }
    public void setvout(double vout){
        this.openLoop = true;
        this.vout = vout;
    }
    public void setOpenLoop(boolean openLoop){
        this.openLoop = openLoop;
    }
    
}
