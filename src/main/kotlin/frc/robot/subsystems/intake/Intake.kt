package frc.robot.subsystems.intake

import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.IntakeConstants.INTAKE_DEPLOYED_ANGLE
import frc.robot.constants.IntakeConstants.INTAKE_EJECT_VOLTAGE
import frc.robot.constants.IntakeConstants.INTAKE_RUN_VOLTAGE
import frc.robot.constants.IntakeConstants.INTAKE_STOWED_ANGLE
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class Intake(
    private val io: IntakeIO
) : SubsystemBase() {

    private val inputs = IntakeIOInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("IO/Intake", inputs)
        Logger.recordOutput("Subsystems/Intake/command", currentCommand?.name ?: "none")

        if (DriverStation.isDisabled()) {
            io.setPivotGoal(inputs.pivotAngle)
        }
    }

    fun intake(): Command {
        return Commands.run(Runnable {
            io.setPivotGoal(INTAKE_DEPLOYED_ANGLE)
            io.setRollerVoltage(INTAKE_RUN_VOLTAGE)
        }, this)
            .finallyDo(Runnable {
                io.setRollerVoltage(Volts.of(0.0))
                io.setPivotGoal(INTAKE_STOWED_ANGLE)
            })
            .withName("intake")
    }

    fun outtake(): Command {
        return Commands.run(Runnable {
            io.setPivotGoal(INTAKE_DEPLOYED_ANGLE)
            io.setRollerVoltage(INTAKE_EJECT_VOLTAGE)
        }, this)
            .finallyDo(Runnable {
                io.setRollerVoltage(Volts.of(0.0))
                io.setPivotGoal(INTAKE_STOWED_ANGLE)
            })
            .withName("outtake")
    }

    fun manualControl(pivot: Supplier<Angle>, roller: Supplier<Voltage>): Command {
        return Commands.run(Runnable {
            io.setPivotGoal(pivot.get())
            io.setRollerVoltage(roller.get())
        }, this)
            .finallyDo(Runnable { io.setRollerVoltage(Volts.of(0.0)) })
            .withName("manual")
    }

    fun voltageControl(pivot: Supplier<Voltage>, roller: Supplier<Voltage>): Command {
        return Commands.run(Runnable {
            io.setPivotVoltage(pivot.get())
            io.setRollerVoltage(roller.get())
        }, this)
            .finallyDo(Runnable { io.setRollerVoltage(Volts.of(0.0)) })
            .withName("manual voltage")
    }

    fun getAngle(): Angle = inputs.pivotAngle
}
