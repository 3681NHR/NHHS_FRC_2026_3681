package frc.robot.commands

import edu.wpi.first.units.Units.Microseconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import org.littletonrobotics.junction.Logger
import java.util.function.BooleanSupplier
import java.util.function.Consumer

/**
 * generic homing command
 */
class HomeCommand(
    private val homeVolts: Voltage,
    private val stopTime: Time,
    private val stoppedSupplier: BooleanSupplier,
    private val voltageConsumer: Consumer<Voltage>,
    private val onHome: Runnable
) : Command() {

    private var stopTimestamp: Double = Double.NaN

    override fun initialize() {
    }

    override fun execute() {
        voltageConsumer.accept(homeVolts)

        if (!stoppedSupplier.asBoolean) {
            stopTimestamp = Double.NaN
        } else if (stopTimestamp.isNaN()) {
            stopTimestamp = Logger.getTimestamp().toDouble()
        }
    }

    override fun end(interrupted: Boolean) {
        if (!interrupted) {
            onHome.run()
        }
        voltageConsumer.accept(Volts.of(0.0))
    }

    override fun isFinished(): Boolean {
        if (stopTimestamp.isNaN()) {
            return false
        }
        if (Microseconds.of(Logger.getTimestamp().toDouble() - stopTimestamp).gte(stopTime)) {
            stopTimestamp = Double.NaN
        }
        return Microseconds.of(Logger.getTimestamp().toDouble() - stopTimestamp).gte(stopTime)
    }
}
