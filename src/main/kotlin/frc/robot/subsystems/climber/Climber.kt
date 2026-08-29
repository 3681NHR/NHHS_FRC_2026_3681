package frc.robot.subsystems.climber

import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.commands.HomeCommand
import frc.robot.constants.ClimberConstants.CLIMBER_HOME_STOP_THRESH
import frc.robot.constants.ClimberConstants.CLIMBER_HOME_STOP_TIME
import frc.robot.constants.ClimberConstants.CLIMBER_HOME_VOLTAGE
import frc.robot.constants.ClimberConstants.CLIMBER_MAX_POSITION
import frc.robot.constants.ClimberConstants.CLIMBER_MIN_POSITION
import frc.utils.HiddenConditionalCommand
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class Climber(
    private val io: ClimberIO
) : SubsystemBase() {

    private val inputs = ClimberIOInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("IO/Climber", inputs)
        Logger.recordOutput("Subsystems/Climber/state", currentCommand?.name ?: "none")
    }

    fun voltageControl(volts: Supplier<Voltage>): Command = run { io.setVoltage(volts.get()) }

    fun positionControl(pos: Supplier<Distance>): Command = run { io.setGoal(pos.get()) }

    fun getPosition(): Distance = inputs.position

    /**
     * reset position to min value and set homed to true
     */
    fun forceHome(): Command = InstantCommand({
        io.setHomed(true)
        io.setPosition(CLIMBER_MIN_POSITION)
    }, this).withName("force home")

    /**
     * uses voltage commands to auto home
     */
    fun home(): Command {
        val c = InstantCommand({ io.setHomed(false) })
            .andThen(
                HomeCommand(
                    CLIMBER_HOME_VOLTAGE,
                    CLIMBER_HOME_STOP_TIME,
                    { CLIMBER_HOME_STOP_THRESH.gte(inputs.velocity) },
                    { v -> io.setVoltage(v) },
                    {
                        io.setPosition(CLIMBER_MIN_POSITION)
                        io.setHomed(true)
                        io.setGoal(CLIMBER_MIN_POSITION)
                    }
                )
            )
        c.addRequirements(this)
        c.setName("auto home")
        return c
    }

    fun extend(): Command = positionControl { CLIMBER_MAX_POSITION }

    fun retract(): Command = positionControl { CLIMBER_MIN_POSITION }

    fun toggle(): Command = HiddenConditionalCommand(
        extend(),
        retract(),
        { inputs.goal.baseUnitMagnitude() <= CLIMBER_MAX_POSITION.baseUnitMagnitude() / 2.0 }
    )
}
