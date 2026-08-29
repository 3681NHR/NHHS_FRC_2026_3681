package frc.robot.oldautos

import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.trajectory.PathPlannerTrajectory
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState
import edu.wpi.first.math.Pair
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import frc.robot.RobotContainer
import frc.robot.commands.DriveToFuel
import frc.robot.constants.DriveConstants
import frc.utils.AllianceUtility
import frc.utils.ExtraMath
import frc.utils.RectZone
import java.util.ArrayList
import java.util.Comparator
import java.util.LinkedList
import edu.wpi.first.units.Units.Radians

/**
 * A factory for creating autonomous programs
 */
class AutoFactory(private val robotContainer: RobotContainer) {

    /* Autonomous program factories
     *
     * Factory methods should be added here for each autonomous program.
     * The factory methods must:
     *   1. Be package-private (i.e. no access modifier)
     *   2. Accept no parameters
     *   3. Return a link Command
     */

    internal fun createIdleAuto(): Pair<PathPlannerTrajectory, Command> {
        return Pair.of(null, IDLE_COMMAND)
    }

    internal fun createExampleAuto(): Pair<PathPlannerTrajectory, Command> {
        return Pair.of(null, Commands.sequence())
    }

    internal fun createRightAIMidAuto(): Pair<PathPlannerTrajectory, Command> {
        try {
            val path = PathPlannerPath.fromChoreoTrajectory("R_to_mid")
            val traj = LinkedList(getTraj(path).states)
            return Pair.of(
                PathPlannerTrajectory(traj),
                Commands.sequence(
                    robotContainer.getDrive().followPath(path),
                    Commands.parallel(
                        robotContainer.getTrackCommand(),
                        robotContainer.fire(),
                        robotContainer.intake(),
                        DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision()) { AllianceUtility.flipRectZone(RectZone(5.7, 0.5, 8.3, 7.6)) }
                    )
                )
            )
        } catch (e: Exception) {
            throw RuntimeException("Failed to create right AI mid Auto", e)
        }
    }

    internal fun createLeftAIMidAuto(): Pair<PathPlannerTrajectory, Command> {
        try {
            val path = PathPlannerPath.fromChoreoTrajectory("L_to_mid")
            val traj = LinkedList(getTraj(path).states)
            return Pair.of(
                PathPlannerTrajectory(traj),
                Commands.sequence(
                    robotContainer.getDrive().followPath(path),
                    Commands.parallel(
                        robotContainer.getTrackCommand(),
                        robotContainer.fire(),
                        robotContainer.intake(),
                        DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision()) { AllianceUtility.flipRectZone(RectZone(5.7, 0.5, 8.3, 7.6)) }
                    )
                )
            )
        } catch (e: Exception) {
            throw RuntimeException("Failed to create left AI mid Auto", e)
        }
    }

    internal fun createLeftAIZoneAuto(): Pair<PathPlannerTrajectory, Command> {
        try {
            return Pair.of(
                null,
                Commands.parallel(
                    robotContainer.getTrackCommand(),
                    robotContainer.fire(),
                    robotContainer.intake(),
                    DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision()) { AllianceUtility.flipRectZone(RectZone(0.669, 4.8, 3.5, 7.42)) }
                )
            )
        } catch (e: Exception) {
            throw RuntimeException("Failed to create left AI zone Auto", e)
        }
    }

    internal fun createDepotAIAuto(): Pair<PathPlannerTrajectory, Command> {
        try {
            val path = PathPlannerPath.fromChoreoTrajectory("depot")
            val traj = getTraj(path).states
            println(PathPlannerTrajectory(traj).totalTimeSeconds)
            return Pair.of(
                PathPlannerTrajectory(traj),
                Commands.parallel(
                    Commands.sequence(
                        robotContainer.getDrive().followPath(path),
                        DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision()) { AllianceUtility.flipRectZone(RectZone(0.669, 4.8, 3.5, 7.42)) }
                    ),
                    robotContainer.getTrackCommand(),
                    Commands.sequence(
                        Commands.waitSeconds(3.0),
                        robotContainer.fire()
                    ),
                    robotContainer.intake()
                )
            )
        } catch (e: Exception) {
            throw RuntimeException("Failed to create left AI zone Auto", e)
        }
    }

    internal fun createBumpDepotAIAuto(): Pair<PathPlannerTrajectory, Command> {
        try {
            val path = PathPlannerPath.fromChoreoTrajectory("bump_depot")
            val traj = LinkedList(getTraj(path).states)
            return Pair.of(
                PathPlannerTrajectory(traj),
                Commands.parallel(
                    Commands.sequence(
                        robotContainer.getDrive().followPath(path),
                        DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision()) { AllianceUtility.flipRectZone(RectZone(0.669, 4.8, 3.5, 7.42)) }
                    ),
                    robotContainer.getTrackCommand(),
                    Commands.sequence(
                        Commands.waitSeconds(3.0),
                        robotContainer.fire()
                    ),
                    robotContainer.intake()
                )
            )
        } catch (e: Exception) {
            throw RuntimeException("Failed to create left AI zone Auto", e)
        }
    }

    internal fun createRightAIZoneAuto(): Pair<PathPlannerTrajectory, Command> {
        try {
            return Pair.of(
                null,
                Commands.parallel(
                    robotContainer.getTrackCommand(),
                    robotContainer.fire(),
                    robotContainer.intake(),
                    DriveToFuel(robotContainer.getDrive(), robotContainer.getFuelVision()) { AllianceUtility.flipRectZone(RectZone(0.669, 0.65, 3.5, 2.7)) }
                )
            )
        } catch (e: Exception) {
            throw RuntimeException("Failed to create left AI zone Auto", e)
        }
    }

    internal fun createPreloadAuto(): Pair<PathPlannerTrajectory, Command> {
        return Pair.of(
            null,
            ParallelCommandGroup(
                robotContainer.getTrackCommand(),
                Commands.waitSeconds(1.0).andThen(robotContainer.fire().alongWith(robotContainer.intake()))
            )
        )
    }

    internal fun createExamplePPAuto(): Pair<PathPlannerTrajectory, Command> {
        try {
            val path = PathPlannerPath.fromPathFile("m4")
            val traj = LinkedList(getTraj(path).states)
            return Pair.of(
                PathPlannerTrajectory(traj),
                Commands.sequence(
                    robotContainer.getDrive().followPath(path)
                )
            )
        } catch (e: Exception) {
            throw RuntimeException("Failed to create Test Auto", e)
        }
    }

    internal fun createDepotAuto(): Pair<PathPlannerTrajectory, Command> {
        try {
            val path = PathPlannerPath.fromChoreoTrajectory("depot")
            val traj = LinkedList(getTraj(path).states)
            return Pair.of(
                PathPlannerTrajectory(traj),
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
                )
            )
        } catch (e: Exception) {
            throw RuntimeException("Failed to create Test Auto", e)
        }
    }

    internal fun createLeftPassAuto(): Pair<PathPlannerTrajectory, Command> {
        try {
            val swipePath = PathPlannerPath.fromChoreoTrajectory("L_swipe")
            val transitionPath = PathPlannerPath.fromChoreoTrajectory("L_mid_to_inner_neutral")
            val depotPath = PathPlannerPath.fromChoreoTrajectory("depot")

            val traj1 = LinkedList(getTraj(swipePath).states)
            val traj2 = LinkedList(getTraj(transitionPath).states)
            val traj3 = LinkedList(getTraj(depotPath).states)

            return Pair.of(
                mergeTrajectories(PathPlannerTrajectory(traj1), PathPlannerTrajectory(traj2), PathPlannerTrajectory(traj3)),
                Commands.sequence(
                    InstantCommand(Runnable { org.littletonrobotics.junction.Logger.recordOutput("auto stage", 0) }),
                    Commands.deadline(
                        robotContainer.getDrive().followPath(swipePath),
                        robotContainer.getTrackCommand(),
                        Commands.sequence(
                            Commands.waitSeconds(1.0),
                            Commands.parallel(
                                robotContainer.intake(),
                                robotContainer.fire()
                            )
                        )
                    ),
                    InstantCommand(Runnable { org.littletonrobotics.junction.Logger.recordOutput("auto stage", 1) }),
                    robotContainer.getDrive().followPath(transitionPath),
                    InstantCommand(Runnable { org.littletonrobotics.junction.Logger.recordOutput("auto stage", 2) }),
                    Commands.parallel(
                        robotContainer.getDrive().followPath(depotPath),
                        robotContainer.getTrackCommand(),
                        Commands.sequence(
                            Commands.waitSeconds(0.8),
                            robotContainer.intake(),
                            robotContainer.fire()
                        )
                    ),
                    InstantCommand(Runnable { org.littletonrobotics.junction.Logger.recordOutput("auto stage", 3) })
                )
            )
        } catch (e: Exception) {
            throw RuntimeException("Failed to create Test Auto", e)
        }
    }

    internal fun createRightPassAuto(): Pair<PathPlannerTrajectory, Command> {
        try {
            val swipePath = PathPlannerPath.fromChoreoTrajectory("R_swipe")
            val transitionPath = PathPlannerPath.fromChoreoTrajectory("R_mid_to_zone")

            val traj1 = LinkedList(getTraj(swipePath).states)
            val traj2 = LinkedList(getTraj(transitionPath).states)

            return Pair.of(
                mergeTrajectories(PathPlannerTrajectory(traj1), PathPlannerTrajectory(traj2)),
                Commands.sequence(
                    InstantCommand(Runnable { org.littletonrobotics.junction.Logger.recordOutput("auto stage", 0) }),
                    Commands.deadline(
                        robotContainer.getDrive().followPath(swipePath),
                        robotContainer.getTrackCommand(),
                        Commands.sequence(
                            Commands.waitSeconds(1.0),
                            Commands.parallel(
                                robotContainer.intake(),
                                robotContainer.fire()
                            )
                        )
                    ),
                    InstantCommand(Runnable { org.littletonrobotics.junction.Logger.recordOutput("auto stage", 1) }),
                    robotContainer.getDrive().followPath(transitionPath),
                    InstantCommand(Runnable { org.littletonrobotics.junction.Logger.recordOutput("auto stage", 2) }),
                    Commands.parallel(
                        robotContainer.getTrackCommand(),
                        robotContainer.intake(),
                        robotContainer.fire()
                    )
                )
            )
        } catch (e: Exception) {
            throw RuntimeException("Failed to create Test Auto", e)
        }
    }

    private fun mergeTrajectories(vararg `in`: PathPlannerTrajectory): PathPlannerTrajectory {
        val traj = LinkedList<PathPlannerTrajectoryState>()
        var timeOffset = 0.0
        for (trajectory in `in`) {
            val states = trajectory.states.toTypedArray()
            val nextOffset = states[states.size - 1].timeSeconds
            for (s in states) {
                s.timeSeconds += timeOffset
                traj.add(s)
            }
            timeOffset += nextOffset
        }
        return PathPlannerTrajectory(traj)
    }

    private fun getTraj(path: PathPlannerPath): PathPlannerTrajectory {
        return (if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) path.flipPath() else path)
            .getIdealTrajectory(DriveConstants.PP_CONFIG).orElse(null)!!
    }

    private fun getAngleToNearestFuel(): Double {
        val robotPos = robotContainer.getDrive().pose.translation
        val fuel: ArrayList<Translation2d> = robotContainer.getFuelVision().getTrackedFuel()
        if (fuel.isNotEmpty()) {
            return ExtraMath.getAngleToPos(selectFuelTarget(robotPos, fuel), robotPos).`in`(Radians)
        }
        return robotContainer.getDrive().pose.rotation.radians
    }

    private fun selectFuelTarget(robotPos: Translation2d, candidates: List<Translation2d>): Translation2d {
        return candidates.stream()
            .min(Comparator.comparingDouble { e: Translation2d -> e.getDistance(robotPos) })
            .orElse(robotPos)
    }

    companion object {
        private val IDLE_COMMAND: Command = Commands.idle()
    }
}
