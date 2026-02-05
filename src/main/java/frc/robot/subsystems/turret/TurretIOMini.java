package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.TurretConstants.*;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.Alert;
import frc.utils.Alert.AlertType;

public class TurretIOMini implements TurretIO {

    private SparkMax motor = new SparkMax(TURRET_MOTOR_ID, SparkLowLevel.MotorType.kBrushless);
    private SparkClosedLoopController controller = motor.getClosedLoopController();
    private CANcoder encoder = new CANcoder(TURRET_ENCODER_1_ID);
    private RelativeEncoder builtinEncoder = motor.getEncoder();
    
    private Angle goal = Radians.of(0.0);
    private boolean openLoop = false;
    private Voltage Vout = Volts.of(0.0);
    private Angle angle = Radians.of(0.0);

    private final LinearSystem<N2, N1, N2> model = LinearSystemId.identifyPositionSystem(TURRET_ID_GAINS.kV(), TURRET_ID_GAINS.kA());
    private final KalmanFilter<N2, N1, N2> filter = new KalmanFilter<N2, N1, N2>(Nat.N2(), Nat.N2(), model, VecBuilder.fill(0.2, 1.0), VecBuilder.fill(1.3, 0.7), 0.02);

    private Alert divergance = new Alert("", AlertType.kWarning);//dynamic text
    private Alert disconenct = new Alert("turret absolute encoder is disconnected", AlertType.kWarning);
    private Alert motorError = new Alert("", AlertType.kError);
    private Alert lost = new Alert("turret internal encoder is not calibrated! \nusing rio control loop and absolute encoder as fallback", AlertType.kError);
    private boolean sync = false;

    public TurretIOMini(){
        if(encoder.isConnected()){
            builtinEncoder.setPosition(encoder.getPosition().getValue().in(Radians));
            sync = true;
        } else {
            lost.set(true);
            disconenct.set(true);
        }
    }

    @Override
    public void updateInputs(TurretIOInputs input){
        disconenct.set(encoder.isConnected());
        motorError.set(motor.getLastError() != REVLibError.kOk);
        motorError.setText("turret motor error: " + motor.getLastError().toString());

        //resync
        if(!sync && encoder.isConnected() && motor.getLastError() != REVLibError.kOk && getSpeed().lt(DegreesPerSecond.of(90))){
            builtinEncoder.setPosition(encoder.getPosition().getValue().in(Radians));
            sync = true;
            lost.set(false);
        }

        if(encoder.getPosition().getValue().isNear(Radians.of(builtinEncoder.getPosition()), TURRET_DIVERGANCE_THRESH)){
            divergance.set(true);
            divergance.setText("turret encoders are divergent by: "+encoder.getPosition().getValue().minus(Radians.of(builtinEncoder.getPosition())).in(Degrees)+" degrees");
        } else {
            divergance.set(false);
        }

        filter.predict(VecBuilder.fill(Vout.in(Volts) - Math.min(TURRET_ID_GAINS.kS(), Math.abs(Vout.in(Volts)))*Math.signum(getSpeed().magnitude())), 0.02);
        filter.correct(VecBuilder.fill(Vout.in(Volts) - Math.min(TURRET_ID_GAINS.kS(), Math.abs(Vout.in(Volts)))*Math.signum(getSpeed().magnitude())), VecBuilder.fill(getAngle().in(Radians), getSpeed().in(RadiansPerSecond)));
        angle = Radians.of(filter.getXhat().get(0,0));

        if(!openLoop){
            motor.getClosedLoopController().setSetpoint(goal.in(Radians), SparkBase.ControlType.kMAXMotionPositionControl);
        } else {
            motor.setVoltage(Vout);
        }
        if(!DriverStation.isEnabled()){
            motor.stopMotor();
        }
        
        input.filteredAngle = angle;
        input.filteredSpeed = RadiansPerSecond.of(filter.getXhat(1));
        input.rawAngle = getAngle();
        input.rawSpeed = getSpeed();

        input.motorVoltageOut = Vout;

        input.goal = goal;
        input.setpointPos = Radians.of(controller.getMAXMotionSetpointPosition());
        input.setpointVel = RadiansPerSecond.of(controller.getMAXMotionSetpointVelocity());
        input.atSetpoint = MathUtil.isNear(goal.in(Radians), angle.in(Radians), TURRET_SETPOINT_TOLERANCE.in(Radians));
    
        input.openLoop = openLoop;
    }

    @Override
    public void setGoal(Angle goal){
        this.openLoop = false;
        this.goal = goal;
    }
    @Override
    public void setVout(Voltage vout){
        this.openLoop = true;
        Vout = vout;
    }
    @Override
    public void setOpenLoop(boolean openLoop){
        this.openLoop = openLoop;
    }
    
    private Angle getAngle(){
        if(sync && !motorError.get()){
            return Radians.of(builtinEncoder.getPosition());
        } else if(!disconenct.get()){
            return encoder.getPosition().getValue();
        } else {
            return Radians.of(builtinEncoder.getPosition());
        }
    }
    private AngularVelocity getSpeed(){
        if(!motorError.get()){
            return RadiansPerSecond.of(builtinEncoder.getVelocity());
        } else {
            return encoder.getVelocity().getValue();
        }
    }
}
