package frc.robot.subsystems.kicker;

import static frc.robot.constants.KickerConstants.*;
import static frc.utils.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.utils.motorWrappers.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

import static edu.wpi.first.units.Units.Amps;

public class KickerIOReal implements KickerIO {

    private final TalonFX motor;
    private final CANrange sensor;

    private final VoltageOut voltageRequest = new VoltageOut(0);

    private Alert motorDisconnectAlert = new Alert("Kicker motor disconnected!", AlertType.kError);
    private Alert sensorDisconnectAlert = new Alert("Kicker CANRange disconnected!", AlertType.kError);

    public KickerIOReal() {
        sensor = new CANrange(KICKER_SENSOR_ID);

        motor = new TalonFX(KICKER_MOTOR_ID);

        // Surface failures rather than silently mis-applying motor inversion,
        // brake mode, or stator current limits.
        tryUntilOk(5, () -> motor.getConfigurator().apply(new MotorOutputConfigs()
                .withInverted(KICKER_MOTOR_INVERT ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake)));

        tryUntilOk(5, () -> motor.getConfigurator().apply(new CurrentLimitsConfigs()
                .withStatorCurrentLimit(KICKER_MAX_CURRENT.in(Amps))
                .withStatorCurrentLimitEnable(true)));
    }

    @Override
    public void updateInputs(KickerIOInputs input) {

        input.speed = motor.getVelocity().getValue();

        input.motorCurrentOut = motor.getStatorCurrent().getValue();
        input.motorVoltageOut = motor.getMotorVoltage().getValue();
        input.motorTemp = motor.getDeviceTemp().getValue();

        input.distance = sensor.getDistance().getValue();
        input.hasBall = input.distance.lte(KICKER_PRELOAD_STOP_DISTANCE);
        input.motorConnected = motor.isConnected();
        input.sensorConnected = sensor.isConnected();

        motorDisconnectAlert.set(!input.motorConnected);
        sensorDisconnectAlert.set(!input.sensorConnected);
    }

    @Override
    public void setVout(Voltage vout) {
        motor.setControl(voltageRequest.withOutput(vout));
    }
}
