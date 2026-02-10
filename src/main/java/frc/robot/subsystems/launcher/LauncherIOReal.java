package frc.robot.subsystems.launcher;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.LauncherConstants.*;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.VoltageConfigs;
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
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class LauncherIOReal implements LauncherIO {

    TalonFX motor = new TalonFX(LAUNCHER_MOTOR_ID);

    VoltageOut openLoopRequest = new VoltageOut(0);
    VelocityVoltage closedLoopRequest = new VelocityVoltage(0);
    
    AngularVelocity goal = RPM.of(0.0);
    Voltage vout = Volts.of(0.0);
    AngularVelocity speed = RPM.of(0.0);
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
        motor.getConfigurator().apply(new VoltageConfigs().withPeakReverseVoltage(0));
    }

    @Override
    public void updateInputs(LauncherIOInputs input){
        filter.predict(VecBuilder.fill(vout.in(Volts) - Math.min(LAUNCHER_ID_GAINS.kS(), Math.abs(vout.in(Volts)))*Math.signum(motor.getVelocity().getValue().in(RadiansPerSecond))), 0.02);
        filter.correct(VecBuilder.fill(vout.in(Volts) - Math.min(LAUNCHER_ID_GAINS.kS(), Math.abs(vout.in(Volts)))*Math.signum(motor.getVelocity().getValue().in(RadiansPerSecond))), VecBuilder.fill(motor.getPosition().getValue().in(Radians), motor.getVelocity().getValue().in(RadiansPerSecond)));
        speed = RadiansPerSecond.of(filter.getXhat().get(1,0));

        if(DriverStation.isEnabled()){
            if(!openLoop){
                motor.setControl(closedLoopRequest.withVelocity(goal));
            } else{
                motor.setControl(openLoopRequest.withOutput(vout));
            }
        } else {
            motor.stopMotor();
        }
        if(motor.getDeviceTemp().getValue().magnitude() > LAUNCHER_MAX_TEMP.magnitude()){
            overheat.set(true);
            overheat.setText("Launcher motor overheat! ("+motor.getDeviceTemp().getValue().in(Celsius)+"C)");
        } else {
            overheat.set(false);
        }
        disconnect.set(motor.isConnected());
        
        input.filteredAngle = Radians.of(filter.getXhat(0));
        input.filteredSpeed = speed;
        input.rawAngle = motor.getPosition().getValue();
        input.rawSpeed = motor.getVelocity().getValue();

        input.motorCurrentOut = motor.getStatorCurrent().getValue();
        input.motorTemp = motor.getDeviceTemp().getValue();
        input.motorVoltageOut = vout;

        input.goal = goal;
        input.atSetpoint = MathUtil.isNear(goal.in(RPM), speed.in(RPM), LAUNCHER_SETPOINT_TOLERANCE.in(RPM));
    
        input.openLoop = openLoop;
    }

    @Override
    public void setGoal(AngularVelocity goal){
        this.openLoop = false;
        this.goal = goal;
    }
    @Override
    public void setVout(Voltage vout){
        this.openLoop = true;
        this.vout = vout;
    }
    @Override
    public void setOpenLoop(boolean openLoop){
        this.openLoop = openLoop;
    }
    
}
