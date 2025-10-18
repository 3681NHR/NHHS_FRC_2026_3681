package frc.robot.autos;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;

/**
 * A factory for creating autonomous programs for a given {@link Auto}
 */
public class AutoFactory {

    private final RobotContainer robotContainer;
    /**
     * Create a new <code>AutoFactory</code>.
     *
     * @param robotContainer The {@link RobotContainer}
     */
    public AutoFactory(final RobotContainer robotContainer) {
        this.robotContainer = robotContainer;
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

    Pair<Pose2d[], Command> createIdleAuto() {
        return Pair.of(new Pose2d[]{}, IDLE_COMMAND);
    }

    Pair<Pose2d[], Command> createExampleAuto() {
        return Pair.of(
                new Pose2d[]{},
                Commands.sequence());
    }
    
    Pair<Pose2d[], Command> createTestAuto() {
        try{
            PathPlannerPath path = PathPlannerPath.fromPathFile("testPath");
        return Pair.of(
                path.getPathPoses().toArray(new Pose2d[0]),
                Commands.sequence(
                    robotContainer.getDrive().followPath(path))
                );
        } catch (Exception e){
            throw new RuntimeException("Failed to create Test Auto", e);
        }
    }

}