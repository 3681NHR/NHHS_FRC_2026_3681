package frc.utils

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import java.util.Collections
import java.util.LinkedHashMap

class HiddenParallelCommandGroup(vararg commands: Command) : Command() {

    private val m_commands: MutableMap<Command, Boolean> = LinkedHashMap()
    private var m_runWhenDisabled = true
    private var m_interruptBehavior: InterruptionBehavior = InterruptionBehavior.kCancelIncoming

    init {
        addCommands(*commands)
    }

    fun addCommands(vararg commands: Command) {
        if (m_commands.containsValue(true)) {
            throw IllegalStateException("Commands cannot be added to a composition while it's running")
        }

        for (command in commands) {
            if (!Collections.disjoint(command.requirements, requirements)) {
                throw IllegalArgumentException(
                    "Multiple commands in a parallel composition cannot require the same subsystems"
                )
            }
            m_commands[command] = true
            m_runWhenDisabled = m_runWhenDisabled and command.runsWhenDisabled()
            if (command.interruptionBehavior == InterruptionBehavior.kCancelSelf) {
                m_interruptBehavior = InterruptionBehavior.kCancelSelf
            }
            CommandScheduler.getInstance().schedule(command)
        }
    }

    override fun initialize() {}

    override fun execute() {}

    override fun end(interrupted: Boolean) {
        if (interrupted) {
            val scheduler = CommandScheduler.getInstance()
            for (command in m_commands.keys) {
                if (scheduler.isScheduled(command)) {
                    scheduler.cancel(command)
                }
            }
        }
    }

    override fun isFinished(): Boolean {
        val scheduler = CommandScheduler.getInstance()
        for (command in m_commands.keys) {
            if (scheduler.isScheduled(command)) {
                return false
            }
        }
        return true
    }

    override fun runsWhenDisabled(): Boolean = m_runWhenDisabled

    override fun getInterruptionBehavior(): InterruptionBehavior = m_interruptBehavior
}
