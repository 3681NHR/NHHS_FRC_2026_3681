package frc.robot.commands

import edu.wpi.first.wpilibj2.command.Command
import java.util.function.Supplier

/**
 * A command that runs another command supplied by a [Supplier]. This is
 * useful for reinitializing a command constructed before calling
 */
class PlayCommand(
    private val cmd: Supplier<Command>
) : Command() {

    override fun initialize() {
        addRequirements(cmd.get().requirements)
        cmd.get().initialize()
    }

    override fun execute() {
        cmd.get().execute()
    }

    override fun end(interrupted: Boolean) {
        cmd.get().end(interrupted)
    }

    override fun isFinished(): Boolean {
        return cmd.get().isFinished
    }
}
