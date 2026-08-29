package frc.utils

import edu.wpi.first.util.ErrorMessages.requireNonNullParam
import edu.wpi.first.util.sendable.SendableBuilder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.Trigger
import java.util.function.BooleanSupplier

class HiddenConditionalCommand(
    private val m_onTrue: Command,
    private val m_onFalse: Command,
    private val m_condition: BooleanSupplier
) : Command() {

    private var m_selectedCommand: Command? = null

    init {
        requireNonNullParam(m_onTrue, "onTrue", "ConditionalCommand")
        requireNonNullParam(m_onFalse, "onFalse", "ConditionalCommand")
        requireNonNullParam(m_condition, "condition", "ConditionalCommand")

        CommandScheduler.getInstance().registerComposedCommands(m_onTrue, m_onFalse)

        addRequirements(m_onTrue.requirements)
        addRequirements(m_onFalse.requirements)

        Trigger(m_condition).onTrue(
            DisabledInstantCommand({
                m_onFalse.end(true)
                m_onTrue.initialize()
            })
        ).onFalse(
            DisabledInstantCommand({
                m_onTrue.end(true)
                m_onFalse.initialize()
            })
        )
    }

    override fun initialize() {
        m_selectedCommand = if (m_condition.asBoolean) m_onTrue else m_onFalse
        m_selectedCommand!!.initialize()
    }

    override fun execute() {
        m_selectedCommand = if (m_condition.asBoolean) m_onTrue else m_onFalse
        m_selectedCommand!!.execute()
    }

    override fun end(interrupted: Boolean) {
        m_selectedCommand = if (m_condition.asBoolean) m_onTrue else m_onFalse
        m_selectedCommand!!.end(interrupted)
    }

    override fun isFinished(): Boolean {
        m_selectedCommand = if (m_condition.asBoolean) m_onTrue else m_onFalse
        return m_selectedCommand!!.isFinished
    }

    override fun runsWhenDisabled(): Boolean =
        m_onTrue.runsWhenDisabled() && m_onFalse.runsWhenDisabled()

    override fun getInterruptionBehavior(): InterruptionBehavior {
        return if (m_onTrue.interruptionBehavior == InterruptionBehavior.kCancelSelf ||
            m_onFalse.interruptionBehavior == InterruptionBehavior.kCancelSelf
        ) {
            InterruptionBehavior.kCancelSelf
        } else {
            InterruptionBehavior.kCancelIncoming
        }
    }

    override fun initSendable(builder: SendableBuilder) {
        super.initSendable(builder)
        builder.addStringProperty("onTrue", m_onTrue::getName, null)
        builder.addStringProperty("onFalse", m_onFalse::getName, null)
        builder.addStringProperty(
            "selected",
            {
                if (m_selectedCommand == null) "null" else m_selectedCommand!!.name
            },
            null
        )
    }

    override fun getName(): String = if (m_condition.asBoolean) m_onTrue.name else m_onFalse.name
}
