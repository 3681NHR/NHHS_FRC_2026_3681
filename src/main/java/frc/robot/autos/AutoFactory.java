package frc.robot.autos;

import java.util.LinkedList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.utils.HiddenParallelCommandGroup;
import org.littletonrobotics.junction.Logger;

/**
 * A factory for creating autonomous programs
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

    Pair<PathPlannerTrajectory, Command> createIdleAuto() {
        return Pair.of(null, IDLE_COMMAND);
    }

    Pair<PathPlannerTrajectory, Command> createExampleAuto() {
        return Pair.of(
                null,
                Commands.sequence());
    }

    Pair<PathPlannerTrajectory, Command> createPreloadAuto() {
        return Pair.of(
                null,
                new ParallelCommandGroup(
                        robotContainer.getTrackCommand(),
                        Commands.waitSeconds(5.0).andThen(robotContainer.fire())
                ));
    }
    
    Pair<PathPlannerTrajectory, Command> createTestAuto() {
        try{
            PathPlannerPath path = PathPlannerPath.fromPathFile("m4");

            List<PathPlannerTrajectoryState> traj = new LinkedList<>(getTraj(path).getStates());

            for(int i=0; i<20-traj.size(); i++){
                traj.add(traj.get(traj.size()-1));
            }
        return Pair.of(
                new PathPlannerTrajectory(traj),
                Commands.sequence(
                    robotContainer.getDrive().followPath(path)
                ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create Test Auto", e);
        }
    }

    Pair<PathPlannerTrajectory, Command> createLeftAuto() {
        try{
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("L trench");

            List<PathPlannerTrajectoryState> traj = new LinkedList<>(getTraj(path).getStates());

            for(int i=0; i<20-traj.size(); i++){
                traj.add(traj.get(traj.size()-1));
            }
            return Pair.of(
                    new PathPlannerTrajectory(traj),
                    Commands.parallel(
                            robotContainer.getDrive().followPath(path),
                            Commands.sequence(
                                Commands.waitSeconds(0.8),
                                new InstantCommand()// TODO: intake
                                        .withTimeout(5),
                                Commands.sequence(
                                        Commands.waitSeconds(0.8),
                                        new InstantCommand()// TODO: shoot
                                                .withTimeout(5)
                                ),
                                Commands.sequence(
                                        Commands.waitSeconds(8.25),
                                        new InstantCommand()// TODO: shoot
                                )
                            )
                    ));
        } catch (Exception e){
            throw new RuntimeException("Failed to create Test Auto", e);
        }
    }

    @SuppressWarnings("unused")
    private PathPlannerTrajectory mergeTrajectories(PathPlannerTrajectory... in){
        List<PathPlannerTrajectoryState> traj = new LinkedList<>();

        double timeOffset = 0.0;
        for (PathPlannerTrajectory trajectory : in) {
            PathPlannerTrajectoryState[] states = trajectory.getStates().toArray(new PathPlannerTrajectoryState[0]);
            double nextoffset = states[states.length - 1].timeSeconds;
            for (PathPlannerTrajectoryState s : states) {
                s.timeSeconds += timeOffset;
                traj.add(s);
            }
            timeOffset += nextoffset;
        }
        return new PathPlannerTrajectory(traj);
    }
    private PathPlannerTrajectory getTraj(PathPlannerPath path){
        return (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? path.flipPath() : path).getIdealTrajectory(DriveConstants.PP_CONFIG).orElse(null);
    }
}