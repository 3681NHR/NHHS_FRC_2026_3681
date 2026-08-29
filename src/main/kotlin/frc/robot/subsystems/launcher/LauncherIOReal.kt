package frc.robot.subsystems.launcher

import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.VoltageConfigs
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.robot.constants.LauncherConstants.LAUNCHER_FF_GAINS
import frc.robot.constants.LauncherConstants.LAUNCHER_MOTOR_ID
import frc.robot.constants.LauncherConstants.LAUNCHER_PID_GAINS
import frc.robot.constants.LauncherConstants.LAUNCHER_SETPOINT_TOLERANCE
import frc.utils.motorWrappers.TalonFX

class LauncherIOReal : LauncherIO {

    private val motor = TalonFX(LAUNCHER_MOTOR_ID)

    private val openLoopRequest = VoltageOut(0.0)
    private val closedLoopRequest = VelocityVoltage(0.0)

    private var openLoop: Boolean = false

    private val disconnect = Alert("Launcher motor disconnected!", Alert.AlertType.kError)

    init {
        motor.configurator.apply(
            Slot0Configs()
                .withKP(LAUNCHER_PID_GAINS.kP)
                .withKI(LAUNCHER_PID_GAINS.kI)
                .withKD(LAUNCHER_PID_GAINS.kD)
                .withKS(LAUNCHER_FF_GAINS.kS)
                .withKV(LAUNCHER_FF_GAINS.kV)
                .withKA(LAUNCHER_FF_GAINS.kA)
        )
        motor.configurator.apply(VoltageConfigs().withPeakReverseVoltage(0.0))
    }

    override fun updateInputs(input: LauncherIO.LauncherIOInputs) {
        disconnect.set(!motor.isConnected)

        input.angle = motor.position.value
        input.speed = motor.velocity.value

        input.motorCurrentOut = motor.statorCurrent.value
        input.motorTemp = motor.deviceTemp.value
        input.motorVoltageOut = motor.motorVoltage.value

        input.goal = closedLoopRequest.velocityMeasure
        input.atSetpoint = MathUtil.isNear(
            closedLoopRequest.velocityMeasure.`in`(RPM),
            motor.velocity.value.`in`(RPM),
            LAUNCHER_SETPOINT_TOLERANCE.`in`(RPM)
        )

        input.openLoop = openLoop
    }

    override fun setGoal(goal: AngularVelocity) {
        openLoop = false
        motor.setControl(closedLoopRequest.withVelocity(goal))
    }

    override fun setVout(vout: Voltage) {
        openLoop = true
        motor.setControl(openLoopRequest.withOutput(vout))
    }
}
