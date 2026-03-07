package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.utils.SparkUtil.tryUntilOk;
import static frc.robot.constants.KickerConstants.*;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.utils.controlWrappers.SimpleFF;

public class KickerIOReal implements KickerIO {

    private final SparkBase kickerSpark;
    private final RelativeEncoder kickerEncoder;
    private final CANrange kickerCANRange;

    private final SparkClosedLoopController kickerController;
    private final SimpleFF kickerFF = new SimpleFF(KICKER_FF_GAINS);

    AngularVelocity goal = RPM.zero();
    Voltage vout = Volts.zero();

    Alert overheatAlert = new Alert("", AlertType.kError);

    private boolean openLoop = false;

    public KickerIOReal() {
        kickerCANRange = new CANrange(KICKER_CAN_RANGE_ID);
        CANrangeConfiguration ballRangeConfig = new CANrangeConfiguration();
        kickerCANRange.getConfigurator().apply(ballRangeConfig);

        kickerSpark = new SparkMax(KICKER_MOTOR_ID, MotorType.kBrushless);
        kickerEncoder = kickerSpark.getEncoder();
        kickerController = kickerSpark.getClosedLoopController();

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.inverted(MOTOR_INVERT).idleMode(IdleMode.kBrake).voltageCompensation(12)
                .smartCurrentLimit((int) KICKER_MAX_CURRENT.in(Amps));
        kickerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(KICKER_PID_GAINS.kP,
                KICKER_PID_GAINS.kI, KICKER_PID_GAINS.kD);

        tryUntilOk(kickerSpark, 5, () -> kickerSpark.configure(kickerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(KickerIOInputs input) {
        input.speed = RPM.of(kickerEncoder.getVelocity());

        input.motorCurrentOut = Amps.of(kickerSpark.getOutputCurrent());
        input.motorVoltageOut = vout;
        input.motorTemp = Celsius.of(kickerSpark.getMotorTemperature());

        input.goal = goal;

        input.openLoop = openLoop;

        input.hasBall = false; // TODO: update so that this actually does something
        input.distance = kickerCANRange.getDistance().getValue();

        if (DriverStation.isEnabled()) {
            if (!openLoop) {
                double ffVoltage = kickerFF.calculate(input.goal.in(RPM));
                kickerController.setSetpoint(goal.in(RPM), ControlType.kVelocity, ClosedLoopSlot.kSlot0,
                        ffVoltage, ArbFFUnits.kVoltage);
            } else {
                kickerController.setSetpoint(vout.in(Volts), ControlType.kVoltage);
            }
        } else {
            kickerSpark.stopMotor();
        }

        if (input.motorTemp.gt(KICKER_MAX_TEMP)) {
            overheatAlert.setText("Kicker motor overheat: " + input.motorTemp.in(Celsius) + " *C !");
            overheatAlert.set(true);
        } else {
            overheatAlert.set(false);
        }
    }

    @Override
    public void setVout(Voltage vout) {
        this.openLoop = true;
        this.vout = vout;
    }

    @Override
    public void setGoal(AngularVelocity goal) {
        this.openLoop = false;
        this.goal = goal;
    }

    @Override
    public void setOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
    }
}
