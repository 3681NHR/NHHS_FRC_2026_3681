package frc.robot.autos;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.*;

/**
 * A factory for creating autonomous programs for a given {@link Auto}
 */
class AutoFactory {
    private final DriverStation.Alliance alliance;

    /**
     * Create a new <code>AutoFactory</code>.
     *
     * @param robotContainer The {@link RobotContainer}
     */
    AutoFactory(final DriverStation.Alliance alliance) {
        this.alliance = alliance;
    }

    /* Autonomous program factories
     *
     * Factory methods should be added here for each autonomous program.
     * The factory methods must:
     *   1. Be package-private (i.e. no access modifier)
     *   2. Accept no parameters
     *   3. Return a link Command
     */
    private static final Command IDLE_COMMAND = Commands.idle();

    Pair<Pose2d, Command> createIdleCommand() {
        return Pair.of(new Pose2d(), IDLE_COMMAND);
    }

    Pair<Pose2d, Command> createJKLABAuto() {
        return Pair.of(
                new Pose2d(),
                Commands.sequence());
    }

}