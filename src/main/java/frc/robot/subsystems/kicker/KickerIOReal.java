package frc.robot.subsystems.kicker;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.utils.SparkUtil.tryUntilOk;
import static frc.robot.constants.KickerConstants.*;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import frc.utils.motorWrappers.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class KickerIOReal implements KickerIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final CANrange sensor;

    private Alert motorDisconnectAlert = new Alert("Kicker motor disconnected!", AlertType.kError);
    private Alert sensorDisconnectAlert = new Alert("Kicker CANRange disconnected!", AlertType.kError);

    public KickerIOReal() {
        sensor = new CANrange(KICKER_SENSOR_ID);

        motor = new SparkMax(KICKER_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.inverted(KICKER_MOTOR_INVERT).idleMode(IdleMode.kBrake).voltageCompensation(12)
                .smartCurrentLimit((int) KICKER_MAX_CURRENT.in(Amps));
        kickerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        kickerConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR).velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        tryUntilOk(motor, 5, () -> motor.configure(kickerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(KickerIOInputs input) {

        input.speed = RPM.of(encoder.getVelocity());

        input.motorCurrentOut = Amps.of(motor.getOutputCurrent());
        input.motorVoltageOut = Volts.of(motor.getAppliedOutput()*motor.getBusVoltage());
        input.motorTemp = Celsius.of(motor.getMotorTemperature());

        input.distance = sensor.getDistance().getValue();
        input.hasBall = input.distance.lte(KICKER_PRELOAD_STOP_DISTANCE);
        input.motorConnected = motor.getLastError() != REVLibError.kCANDisconnected;
        input.sensorConnected = sensor.isConnected();

        motorDisconnectAlert.set(!input.motorConnected);
        sensorDisconnectAlert.set(!input.motorConnected);
    }

    @Override
    public void setVout(Voltage vout) {
        motor.setVoltage(vout);
    }
}
