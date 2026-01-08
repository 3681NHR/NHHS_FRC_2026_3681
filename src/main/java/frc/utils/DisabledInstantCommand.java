package frc.utils;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

/*
 * instantCommand that can run while disabled
 */
public class DisabledInstantCommand extends InstantCommand {
    public DisabledInstantCommand(Runnable toRun, Subsystem... requirements) {
        super(toRun, requirements);
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
