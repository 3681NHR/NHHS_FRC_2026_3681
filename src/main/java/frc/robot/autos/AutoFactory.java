package frc.robot.autos;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedList;
import java.util.List;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveConstants;
import frc.utils.ExtraMath;

import static edu.wpi.first.units.Units.Radians;

/**
 * A factory for creating autonomous programs
 */
public class AutoFactory {

    private final RobotContainer robotContainer;
    private Translation2d smoothedFuelTarget;
    private double smoothedFuelAngleRad;
    private double lastFuelUpdateSec = Double.NEGATIVE_INFINITY;

    private static final double TARGET_SMOOTHING_ALPHA = 0.25;
    private static final double SWITCH_DISTANCE_BIAS = 0.6;
    private static final double MAX_ANGLE_STEP_RAD = Units.degreesToRadians(7.0);
    private static final double LOST_TARGET_HOLD_SEC = 0.25;
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

    Pair<PathPlannerTrajectory, Command> createAIAuto() {
        return Pair.of(
                null,
                Commands.parallel(
//                        robotContainer.getTrackCommand(),
//                        robotContainer.fire(),
                        robotContainer.intake(),
                        robotContainer.getDrive().rotationLock(this::getAngleToNearestFuel, () -> 0.5*Math.cos(robotContainer.getDrive().getPose().getRotation().getRadians()), () -> 0.5*Math.sin(robotContainer.getDrive().getPose().getRotation().getRadians()))
                ));
    }

    Pair<PathPlannerTrajectory, Command> createPreloadAuto() {
        return Pair.of(
                null,
                new ParallelCommandGroup(
                        robotContainer.getTrackCommand(),
                        Commands.waitSeconds(1.0).andThen(robotContainer.fire().alongWith(robotContainer.intake()))
                ));
    }
    
    Pair<PathPlannerTrajectory, Command> createExamplePPAuto() {
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

    Pair<PathPlannerTrajectory, Command> createDepotAuto() {
        try{
            PathPlannerPath path = PathPlannerPath.fromChoreoTrajectory("depot");

            List<PathPlannerTrajectoryState> traj = new LinkedList<>(getTraj(path).getStates());

            for(int i=0; i<20-traj.size(); i++){
                traj.add(traj.get(traj.size()-1));
            }
            return Pair.of(
                    new PathPlannerTrajectory(traj),
                    Commands.parallel(
                            robotContainer.getDrive().followPath(path),
                            robotContainer.getTrackCommand(),
                            Commands.sequence(
                                Commands.waitSeconds(0.8),
                                robotContainer.intake()
                            ),
                            Commands.sequence(
                                    Commands.waitSeconds(0.8),
                                    robotContainer.fire()
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

    private double getAngleToNearestFuel(){
        Translation2d robotPos = robotContainer.getDrive().getPose().getTranslation();
        ArrayList<Translation2d> fuel = robotContainer.getFuelVision().getTrackedFuel();

        if (!fuel.isEmpty()) {
            Translation2d selected = selectFuelTarget(robotPos, fuel);
            if (smoothedFuelTarget == null) {
                smoothedFuelTarget = selected;
            } else {
                smoothedFuelTarget = smoothedFuelTarget.interpolate(selected, TARGET_SMOOTHING_ALPHA);
            }

            double desiredAngle = ExtraMath.getAngleToPos(smoothedFuelTarget, robotPos).in(Radians);
            if (lastFuelUpdateSec == Double.NEGATIVE_INFINITY) {
                smoothedFuelAngleRad = desiredAngle;
            } else {
                double angleError = MathUtil.angleModulus(desiredAngle - smoothedFuelAngleRad);
                smoothedFuelAngleRad = MathUtil.angleModulus(
                        smoothedFuelAngleRad + MathUtil.clamp(angleError, -MAX_ANGLE_STEP_RAD, MAX_ANGLE_STEP_RAD));
            }

            lastFuelUpdateSec = Timer.getFPGATimestamp();
            return smoothedFuelAngleRad;
        }

        if (Timer.getFPGATimestamp() - lastFuelUpdateSec <= LOST_TARGET_HOLD_SEC) {
            return smoothedFuelAngleRad;
        }

        smoothedFuelTarget = null;
        return robotContainer.getDrive().getPose().getRotation().getRadians();
    }

    private Translation2d selectFuelTarget(Translation2d robotPos, List<Translation2d> candidates) {
        if (smoothedFuelTarget == null) {
            return candidates.stream()
                    .min(Comparator.comparingDouble(e -> e.getDistance(robotPos)))
                    .orElse(robotPos);
        }

        return candidates.stream()
                .min(Comparator.comparingDouble(e -> e.getDistance(smoothedFuelTarget) + SWITCH_DISTANCE_BIAS * e.getDistance(robotPos)))
                .orElse(candidates.get(0));
    }
}