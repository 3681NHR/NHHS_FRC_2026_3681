package frc.robot.oldautos

import com.pathplanner.lib.trajectory.PathPlannerTrajectory
import edu.wpi.first.math.Pair
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import java.util.function.Function

/**
 * An autonomous program.
 */
class AutoProgram(
    private val label: String,
    private val commandFactory: Function<AutoFactory, Pair<PathPlannerTrajectory, Command>>
) {
    private var commandPair: Pair<PathPlannerTrajectory, Command>? = null

    fun update(factory: AutoFactory) {
        commandPair = commandFactory.apply(factory)
    }

    /**
     * Get the label for this program
     * @return
     * The label for this program
     */
    fun getLabel(): String = label

    /**
     * Construct the [Command] for this program from the provided [AutoFactory]
     *
     * @param autoFactory
     * The [AutoFactory] to use when creating the [Command]
     * @return
     * The [Command] for this program from the provided [AutoFactory]
     */
    fun getCommand(autoFactory: AutoFactory): Command = commandPair!!.second

    fun getStartingPose(autoFactory: AutoFactory): Pose2d {
        return if (getPoses(autoFactory).isEmpty()) {
            Pose2d()
        } else {
            commandPair!!.first.initialPose
        }
    }

    fun getPoses(autoFactory: AutoFactory): Array<Pose2d> {
        val traj = commandPair?.first
        return if (traj != null) {
            traj.states.map { it.pose }.toTypedArray()
        } else {
            emptyArray()
        }
    }

    fun getPoseAtTime(autoFactory: AutoFactory, time: Double): Pose2d {
        val traj = commandPair?.first
        return traj?.sample(time)?.pose ?: Pose2d()
    }

    fun getPathLength(autoFactory: AutoFactory): Double {
        return commandPair?.first?.totalTimeSeconds ?: 0.0
    }
}
