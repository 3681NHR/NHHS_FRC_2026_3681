package frc.robot.subsystems.indexer

import com.revrobotics.PersistMode
import com.revrobotics.REVLibError
import com.revrobotics.ResetMode
import com.revrobotics.spark.FeedbackSensor
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.robot.constants.IndexerConstants.INDEXER_MAX_CURRENT
import frc.robot.constants.IndexerConstants.INDEXER_MOTOR_ID
import frc.robot.constants.IndexerConstants.INDEXER_MOTOR_INVERT
import frc.robot.constants.IndexerConstants.POSITION_CONVERSION_FACTOR
import frc.robot.constants.IndexerConstants.VELOCITY_CONVERSION_FACTOR
import frc.utils.SparkUtil.tryUntilOk
import frc.utils.motorWrappers.SparkMax

class IndexerIOReal : IndexerIO {

    private val motor = SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushless)
    private val encoder = motor.getEncoder()

    private val motorDisconnectAlert = Alert("Indexer motor disconnected!", Alert.AlertType.kError)

    init {
        val kickerConfig = SparkMaxConfig()
        kickerConfig.inverted(INDEXER_MOTOR_INVERT).idleMode(IdleMode.kBrake).voltageCompensation(12.0)
            .smartCurrentLimit(INDEXER_MAX_CURRENT.`in`(Amps).toInt())
        kickerConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        kickerConfig.encoder.positionConversionFactor(POSITION_CONVERSION_FACTOR)
            .velocityConversionFactor(VELOCITY_CONVERSION_FACTOR)

        tryUntilOk(motor, 5) {
            motor.configure(kickerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
        }
    }

    override fun updateInputs(input: IndexerIO.IndexerIOInputs) {
        input.speed = RPM.of(encoder.velocity)

        input.motorCurrentOut = Amps.of(motor.outputCurrent)
        input.motorVoltageOut = Volts.of(motor.appliedOutput * motor.busVoltage)
        input.motorTemp = Celsius.of(motor.motorTemperature)

        input.motorConnected = motor.getLastError() != REVLibError.kCANDisconnected

        motorDisconnectAlert.set(!input.motorConnected)
    }

    override fun setVout(vout: Voltage) {
        motor.setVoltage(vout)
    }
}
