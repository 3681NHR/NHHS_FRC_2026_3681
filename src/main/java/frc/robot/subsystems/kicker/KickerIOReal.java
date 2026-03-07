package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.utils.SparkUtil.tryUntilOk;
import static frc.robot.constants.KickerConstants.*;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import frc.utils.motorWrappers.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utils.controlWrappers.SimpleFF;

public class KickerIOReal implements KickerIO {

    private final SparkMax kickerSpark;
    private final RelativeEncoder kickerEncoder;
    private final CANrange kickerCANRange;

    private final SimpleFF kickerFF = new SimpleFF(KICKER_FF_GAINS);

    LinearVelocity goal = MetersPerSecond.zero();
    Voltage vout = Volts.zero();

    private boolean openLoop = false;

    public KickerIOReal() {
        kickerCANRange = new CANrange(KICKER_CAN_RANGE_ID);

        kickerSpark = new SparkMax(KICKER_MOTOR_ID, MotorType.kBrushless);
        kickerEncoder = kickerSpark.getEncoder();

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.inverted(MOTOR_INVERT).idleMode(IdleMode.kBrake).voltageCompensation(12)
                .smartCurrentLimit((int) KICKER_MAX_CURRENT.in(Amps));
        kickerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        kickerConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR).velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        tryUntilOk(kickerSpark, 5, () -> kickerSpark.configure(kickerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));
    }

    @Override
    public void updateInputs(KickerIOInputs input) {
        input.speed = MetersPerSecond.of(kickerEncoder.getVelocity());

        input.motorCurrentOut = Amps.of(kickerSpark.getOutputCurrent());
        input.motorVoltageOut = vout;
        input.motorTemp = Celsius.of(kickerSpark.getMotorTemperature());

        input.goal = goal;

        input.openLoop = openLoop;

        input.hasBall = false; // TODO: update so that this actually does something
        input.distance = kickerCANRange.getDistance().getValue();

        if (DriverStation.isEnabled()) {
            if (!openLoop) {
                double ffVoltage = kickerFF.calculate(input.goal.in(MetersPerSecond));
                kickerSpark.setVoltage(ffVoltage);
            } else {
                kickerSpark.setVoltage(vout);
            }
        } else {
            kickerSpark.stopMotor();
        }
    }

    @Override
    public void setVout(Voltage vout) {
        this.openLoop = true;
        this.vout = vout;
    }

    @Override
    public void setGoal(LinearVelocity goal) {
        this.openLoop = false;
        this.goal = goal;
    }
}
