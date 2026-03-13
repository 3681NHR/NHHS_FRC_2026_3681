package frc.robot.subsystems.indexer;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.utils.motorWrappers.SparkMax;

import static edu.wpi.first.units.Units.*;
import static frc.robot.constants.IndexerConstants.*;
import static frc.utils.SparkUtil.tryUntilOk;

public class IndexerIOReal implements IndexerIO {

    private final SparkMax motor;
    private final RelativeEncoder encoder;

    private final Alert motorDisconnectAlert = new Alert("Indexer motor disconnected!", AlertType.kError);

    public IndexerIOReal() {

        motor = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkMaxConfig kickerConfig = new SparkMaxConfig();
        kickerConfig.inverted(INDEXER_MOTOR_INVERT).idleMode(IdleMode.kBrake).voltageCompensation(12)
                .smartCurrentLimit((int) INDEXER_MAX_CURRENT.in(Amps));
        kickerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        kickerConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR).velocityConversionFactor(VELOCITY_CONVERSION_FACTOR);

        tryUntilOk(motor, 5, () -> motor.configure(kickerConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters));

    }

    @Override
    public void updateInputs(IndexerIOInputs input) {

        input.speed = RPM.of(encoder.getVelocity());

        input.motorCurrentOut = Amps.of(motor.getOutputCurrent());
        input.motorVoltageOut = Volts.of(motor.getAppliedOutput()*motor.getBusVoltage());
        input.motorTemp = Celsius.of(motor.getMotorTemperature());

        input.motorConnected = motor.getLastError() != REVLibError.kCANDisconnected;

        motorDisconnectAlert.set(!input.motorConnected);
    }

    @Override
    public void setVout(Voltage vout) {
        motor.setVoltage(vout);
    }
}
