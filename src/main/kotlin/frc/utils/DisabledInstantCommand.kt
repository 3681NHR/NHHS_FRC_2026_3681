package frc.utils

import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.Subsystem

class DisabledInstantCommand(toRun: Runnable, vararg requirements: Subsystem) : InstantCommand(toRun, *requirements) {
    override fun runsWhenDisabled(): Boolean = true
}
