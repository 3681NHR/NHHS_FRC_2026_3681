package frc.robot.subsystems.indexer

import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.constants.IndexerConstants.INDEXER_FEED_VOLTAGE
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

class Indexer(
    private val io: IndexerIO
) : SubsystemBase() {

    private val inputs = IndexerIOInputsAutoLogged()

    override fun periodic() {
        io.updateInputs(inputs)
        Logger.processInputs("IO/Indexer", inputs)
        Logger.recordOutput("Subsystems/Indexer/state", currentCommand?.name ?: "none")
    }

    fun feed(): Command {
        return Commands.run(Runnable {
            io.setVout(INDEXER_FEED_VOLTAGE)
        }, this).withName("Feed")
    }

    fun reverse(): Command {
        return Commands.run(Runnable {
            io.setVout(INDEXER_FEED_VOLTAGE.unaryMinus())
        }, this).withName("Feed")
    }

    fun stop(): Command {
        return Commands.run(Runnable {
            io.setVout(Volts.of(0.0))
        }, this).withName("Stop")
    }

    fun voltageControl(volt: Supplier<Voltage>): Command {
        return Commands.run(Runnable {
            io.setVout(volt.get())
        }, this).withName("Voltage Control")
    }
}
