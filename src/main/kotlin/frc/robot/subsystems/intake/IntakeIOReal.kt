package frc.robot.subsystems.intake

import com.ctre.phoenix6.hardware.CANcoder
import com.revrobotics.PersistMode
import com.revrobotics.REVLibError
import com.revrobotics.ResetMode
import com.revrobotics.spark.SparkLowLevel.MotorType
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode
import com.revrobotics.spark.config.SparkMaxConfig
import edu.wpi.first.units.Units
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.Alert
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_CURRENT_LIM
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_ENCODER_ID
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_FF_GAINS
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_INVERTED
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_MOTOR_ID
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_PID_GAINS
import frc.robot.constants.IntakeConstants.INTAKE_PIVOT_TOLERANCE
import frc.robot.constants.IntakeConstants.INTAKE_ROLLER_CURRENT_LIM
import frc.robot.constants.IntakeConstants.INTAKE_ROLLER_INVERTED
import frc.robot.constants.IntakeConstants.INTAKE_ROLLER_MOTOR_ID
import frc.robot.constants.IntakeConstants.INTAKE_STOWED_ANGLE
import frc.utils.controlWrappers.ArmFF
import frc.utils.controlWrappers.ProfiledPID
import frc.utils.motorWrappers.SparkMax

class IntakeIOReal : IntakeIO {

    // Roller
    private val rollerMotor = SparkMax(INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless)
    private val rollerEncoder = rollerMotor.getEncoder()

    private val rollerMotorDisconnect = Alert("Intake roller Spark disconnected!", Alert.AlertType.kError)

    // Pivot
    private val pivotMotor = SparkMax(INTAKE_PIVOT_MOTOR_ID, MotorType.kBrushless)
    private val pivotEncoder = CANcoder(INTAKE_PIVOT_ENCODER_ID)

    private val pivotPID = ProfiledPID(INTAKE_PIVOT_PID_GAINS)
    private var pivotFF = ArmFF(INTAKE_PIVOT_FF_GAINS)

    private val pivotMotorDisconnect = Alert("Intake pivot Spark disconnected!", Alert.AlertType.kError)
    private val pivotEncoderDisconnect = Alert("Intake pivot encoder disconnected!", Alert.AlertType.kError)

    private var pivotOpenLoop = false
    private var pivotGoal: Angle = INTAKE_STOWED_ANGLE

    init {
        // Live-tuning callbacks
        INTAKE_PIVOT_PID_GAINS.withCallback { pivotPID.setGains(INTAKE_PIVOT_PID_GAINS) }
        INTAKE_PIVOT_FF_GAINS.withCallback {
            pivotFF.setKs(INTAKE_PIVOT_FF_GAINS.kS)
            pivotFF.setKg(INTAKE_PIVOT_FF_GAINS.kG)
            pivotFF.setKv(INTAKE_PIVOT_FF_GAINS.kV)
            pivotFF.setKa(INTAKE_PIVOT_FF_GAINS.kA)
        }

        pivotPID.setTolerance(INTAKE_PIVOT_TOLERANCE.`in`(Units.Radians))

        // Roller motor config
        val rollerCfg = SparkMaxConfig()
        rollerCfg.idleMode(IdleMode.kBrake)
            .inverted(INTAKE_ROLLER_INVERTED)
            .smartCurrentLimit(INTAKE_ROLLER_CURRENT_LIM.`in`(Amps).toInt())
        rollerMotor.configure(rollerCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)

        // Pivot motor config
        val pivotCfg = SparkMaxConfig()
        pivotCfg.idleMode(IdleMode.kBrake)
            .inverted(INTAKE_PIVOT_INVERTED)
            .smartCurrentLimit(INTAKE_PIVOT_CURRENT_LIM.`in`(Amps).toInt())
        pivotMotor.configure(pivotCfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters)
    }

    override fun updateInputs(input: IntakeIO.IntakeIOInputs) {
        // Roller closed-loop
        input.rollerVelocity = Units.RPM.of(rollerEncoder.velocity)

        input.rollerVoltageOut = Units.Volts.of(rollerMotor.appliedOutput * rollerMotor.busVoltage)
        input.rollerCurrentOut = Units.Amps.of(rollerMotor.outputCurrent)
        input.rollerTemp = Units.Celsius.of(rollerMotor.motorTemperature)

        input.rollerConnected = rollerMotor.getLastError() != REVLibError.kCANDisconnected
        rollerMotorDisconnect.set(!input.rollerConnected)

        // Pivot closed-loop
        if (!pivotOpenLoop) {
            val pid = pivotPID.calculate(pivotEncoder.absolutePosition.value.`in`(Units.Radians), pivotGoal.`in`(Units.Radians))
            val ff = pivotFF.calculate(pivotEncoder.absolutePosition.value.`in`(Radians), pivotPID.setpoint.velocity)
            pivotMotor.setVoltage(Units.Volts.of(pid + ff))
        }
        input.pivotAngle = pivotEncoder.absolutePosition.value
        input.pivotVelocity = pivotEncoder.velocity.value

        input.pivotGoal = pivotGoal
        input.pivotSetpointPos = Units.Radians.of(pivotPID.setpoint.position)
        input.pivotSetpointVel = Units.RadiansPerSecond.of(pivotPID.setpoint.velocity)
        input.pivotAtSetpoint = pivotPID.atSetpoint()

        input.pivotVoltageOut = Units.Volts.of(pivotMotor.appliedOutput * pivotMotor.busVoltage)
        input.pivotCurrentOut = Units.Amps.of(pivotMotor.outputCurrent)
        input.pivotTemp = Units.Celsius.of(pivotMotor.motorTemperature)

        input.pivotMotorConnected = pivotMotor.getLastError() != REVLibError.kCANDisconnected
        input.pivotEncoderConnected = pivotEncoder.isConnected()
        input.pivotOpenLoop = pivotOpenLoop
        pivotMotorDisconnect.set(!input.pivotMotorConnected)
        pivotEncoderDisconnect.set(!input.pivotEncoderConnected)
    }

    override fun setRollerVoltage(voltage: Voltage) {
        rollerMotor.setVoltage(voltage)
    }

    override fun setPivotGoal(goal: Angle) {
        pivotOpenLoop = false
        pivotGoal = goal
    }

    override fun setPivotVoltage(voltage: Voltage) {
        pivotOpenLoop = true
        pivotMotor.setVoltage(voltage)
    }
}
