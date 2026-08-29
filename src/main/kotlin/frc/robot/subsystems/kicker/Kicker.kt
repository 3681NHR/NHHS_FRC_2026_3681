package frc.robot.subsystems.kicker

import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.KickerConstants.KICKER_FEED_VOLTAGE
import frc.robot.constants.KickerConstants.KICKER_UNLOAD_MAX_DISTANCE
import frc.robot.constants.KickerConstants.KICKER_UNLOAD_PARTIAL_VOLTAGE
import frc.robot.constants.KickerConstants.KICKER_UNLOAD_VOLTAGE
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import java.util.function.Supplier

class Kicker(
    private val io: KickerIO
) : SubsystemBase() {

    private val inputs = KickerIOInputsAutoLogged()

    private val unloadEnabled = LoggedNetworkBoolean("Overrides/Kicker unload", true)

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("IO/Kicker", inputs)
        Logger.recordOutput("Subsystems/Kicker/state", currentCommand?.name ?: "none")
    }

    fun hold(): Command {
        return Commands.run(Runnable {
            if (unloadEnabled.get()) {
                if (inputs.distance.lte(KICKER_UNLOAD_MAX_DISTANCE) && inputs.sensorConnected) {
                    io.setVout(KICKER_UNLOAD_VOLTAGE)
                } else {
                    io.setVout(KICKER_UNLOAD_PARTIAL_VOLTAGE) // fuel may be in deadzone, prevents wasting power by running lower
                }
            } else {
                io.setVout(Volts.zero())
            }
        }, this).withName("Hold")
    }

    fun feed(): Command {
        return Commands.run(Runnable {
            io.setVout(KICKER_FEED_VOLTAGE)
        }, this).withName("Feed")
    }

    fun reverse(): Command {
        return Commands.run(Runnable {
            io.setVout(KICKER_FEED_VOLTAGE.unaryMinus())
        }, this).withName("Feed")
    }

    fun voltageControl(volt: Supplier<Voltage>): Command {
        return Commands.run(Runnable {
            io.setVout(volt.get())
        }, this).withName("Voltage Control")
    }
}
