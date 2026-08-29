package frc.robot.subsystems.kicker

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANrange
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.robot.constants.KickerConstants.KICKER_MAX_CURRENT
import frc.robot.constants.KickerConstants.KICKER_MOTOR_ID
import frc.robot.constants.KickerConstants.KICKER_MOTOR_INVERT
import frc.robot.constants.KickerConstants.KICKER_PRELOAD_STOP_DISTANCE
import frc.robot.constants.KickerConstants.KICKER_SENSOR_ID
import frc.utils.PhoenixUtil.tryUntilOk
import frc.utils.motorWrappers.TalonFX

class KickerIOReal : KickerIO {

    private val motor = TalonFX(KICKER_MOTOR_ID)
    private val sensor = CANrange(KICKER_SENSOR_ID)

    private val voltageRequest = VoltageOut(0.0)

    private val motorDisconnectAlert = Alert("Kicker motor disconnected!", Alert.AlertType.kError)
    private val sensorDisconnectAlert = Alert("Kicker CANRange disconnected!", Alert.AlertType.kError)

    init {
        // Surface failures rather than silently mis-applying motor inversion,
        // brake mode, or stator current limits.
        tryUntilOk(5) {
            motor.configurator.apply(
                MotorOutputConfigs()
                    .withInverted(
                        if (KICKER_MOTOR_INVERT) InvertedValue.Clockwise_Positive
                        else InvertedValue.CounterClockwise_Positive
                    )
                    .withNeutralMode(NeutralModeValue.Brake)
            )
        }

        tryUntilOk(5) {
            motor.configurator.apply(
                CurrentLimitsConfigs()
                    .withStatorCurrentLimit(KICKER_MAX_CURRENT.`in`(Amps))
                    .withStatorCurrentLimitEnable(true)
            )
        }
    }

    override fun updateInputs(input: KickerIO.KickerIOInputs) {
        input.speed = motor.velocity.value

        input.motorCurrentOut = motor.statorCurrent.value
        input.motorVoltageOut = motor.motorVoltage.value
        input.motorTemp = motor.deviceTemp.value

        input.distance = sensor.distance.value
        input.hasBall = input.distance.lte(KICKER_PRELOAD_STOP_DISTANCE)
        input.motorConnected = motor.isConnected
        input.sensorConnected = sensor.isConnected

        motorDisconnectAlert.set(!input.motorConnected)
        sensorDisconnectAlert.set(!input.sensorConnected)
    }

    override fun setVout(vout: Voltage) {
        motor.setControl(voltageRequest.withOutput(vout))
    }
}
