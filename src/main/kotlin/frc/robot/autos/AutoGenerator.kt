package frc.robot.autos

import com.pathplanner.lib.trajectory.PathPlannerTrajectory
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState
import edu.wpi.first.math.MathUtil
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.RobotContainer
import frc.utils.AllianceUtility
import frc.utils.LoggedField2d
import frc.utils.Periodic
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import java.util.LinkedList

class AutoGenerator(private val container: RobotContainer) : Periodic() {

    private val sequence: LinkedList<Path> = LinkedList()
    private var chooser: LoggedDashboardChooser<Path> = LoggedDashboardChooser("auto/selector")
    private val enter: LoggedNetworkBoolean = LoggedNetworkBoolean("SmartDashboard/auto/enter", false)
    private val back: LoggedNetworkBoolean = LoggedNetworkBoolean("SmartDashboard/auto/back", false)
    private val timeSelector: LoggedNetworkNumber = LoggedNetworkNumber("SmartDashboard/auto/play bar", 0.0)
    private val animate: LoggedNetworkBoolean = LoggedNetworkBoolean("SmartDashboard/auto/play\\pause", false)
    private val field: LoggedField2d = LoggedField2d()

    init {
        populateChooser(Path.START, true)
        generateText()
        SmartDashboard.putData("auto/map", field)
    }

    override fun update() {
        if (animate.get()) {
            timeSelector.set(MathUtil.inputModulus(timeSelector.get() + (0.02 / getTotalTime()), 0.0, 1.0))
        }
        if (enter.getAsBoolean() && chooser.get() != null) {
            enter(chooser.get())
            enter.set(false)
        }
        if (back.getAsBoolean()) {
            if (sequence.isNotEmpty()) {
                sequence.removeLast()
            }
            if (sequence.isNotEmpty()) {
                populateChooser(sequence.last, false)
            } else {
                populateChooser(Path.START, true)
            }
            generateText()
            back.set(false)
        }
        field.getObject("traj").setPoses(
            sequence.mapNotNull { it.ppPath }
                .filter { it.isNotEmpty() }
                .flatMap { it }
                .map { it.pose }
        )
        field.setRobotPose(getPoseAtTime(timeSelector.get() * getTotalTime()))
        if (sequence.isNotEmpty() && sequence.last.ppPath != null) {
            field.getObject("endPose").setPose(sequence.last.ppPath!!.last.pose)
        }
        field.getObject("currPose").setPose(
            AllianceUtility.flipPose(
                container.getDrive().pose.rotateAround(
                    AllianceUtility.FIELD_CENTER_POINT.translation,
                    Rotation2d.k180deg
                )
            )
        )
        Logger.recordOutput("auto/selected time", timeSelector.get() * getTotalTime())
    }

    private fun enter(e: Path) {
        if (chooser.get() != null) {
            sequence.add(chooser.get())
            populateChooser(chooser.get(), false)
            generateText()
        }
    }

    private fun populateChooser(point: Path, startingPoint: Boolean) {
        chooser.getSendableChooser().close()
        chooser = LoggedDashboardChooser("auto/selector")
        for (p in point.options) {
            if (startingPoint) {
                chooser.addOption("start at ${p.end}", p)
            } else {
                chooser.addOption("${p.end} via ${p.displayName}", p)
            }
        }
    }

    private fun generateText() {
        if (sequence.isNotEmpty()) {
            val out = StringBuilder(sequence.first.end)
            if (sequence.size > 1) {
                val it = sequence.listIterator(1)
                while (it.hasNext()) {
                    val p = it.next()
                    out.append(String.format(" =>(%s) %s", p.displayName, p.end))
                }
            }
            Logger.recordOutput("auto/generated", out.toString())
        } else {
            Logger.recordOutput("auto/generated", "select a starting point first!")
        }
    }

    @AutoLogOutput(key = "auto/estimated time")
    private fun getTotalTime(): Double {
        if (sequence.size <= 1) {
            return 0.0
        }
        val trajectories = sequence.mapNotNull { it.ppPath }
            .filter { it.isNotEmpty() }
            .map { PathPlannerTrajectory(it) }
            .toTypedArray()
        return mergeTrajectories(*trajectories).totalTimeSeconds
    }

    private fun getPoseAtTime(time: Double): Pose2d {
        if (sequence.size <= 1) {
            if (sequence.isEmpty()) {
                return Pose2d()
            }
            // NOTE: this assumes the starting points all have at least 1 possible next path
            return sequence.first.options[0].ppPath!!.first.pose
        }
        val trajectories = sequence.mapNotNull { it.ppPath }
            .filter { it.isNotEmpty() }
            .map { PathPlannerTrajectory(it) }
            .toTypedArray()
        return mergeTrajectories(*trajectories).sample(time).pose
    }

    private fun mergeTrajectories(vararg `in`: PathPlannerTrajectory): PathPlannerTrajectory {
        val traj = LinkedList<PathPlannerTrajectoryState>()
        var timeOffset = 0.0
        for (trajectory in `in`) {
            val states = trajectory.states.toTypedArray()
            val nextOffset = if (states.isNotEmpty()) states.last().timeSeconds else 0.0
            for (s in states) {
                traj.add(s.copyWithTime(s.timeSeconds + timeOffset))
            }
            timeOffset += nextOffset
        }
        return PathPlannerTrajectory(traj)
    }

    fun getCommand(): Command {
        return Commands.sequence(*sequence.map { it.Command.get() }.toTypedArray())
    }
}
