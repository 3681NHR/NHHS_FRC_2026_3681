package frc.robot.commands

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.constants.DriveConstants
import frc.robot.subsystems.swerve.Drive
import org.littletonrobotics.junction.Logger
import java.util.function.Supplier

/**
 * use pathplanner PID to continue driving to target after the main path is
 * finished
 *
 * note: this command does not time out on its own, it is recommended to use a
 * timeout decorator
 *
 * @see <a href="https://docs.google.com/document/d/10if4xjAaETTceUVn7l4J-jOCOnm5CJUDS5RAVNIJMQM/edit?tab=t.0">spartronics whitepaper on auto align</a>
 */
class FineTuneAlign(
    private val target: Supplier<Pose2d>,
    private val drive: Drive
) : Command() {

    private lateinit var state: PathPlannerTrajectoryState
    private var done: Boolean = false

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        state = PathPlannerTrajectoryState()
        state.pose = target.get()
    }

    override fun execute() {
        drive.runVelocity(drive.autoController.calculateRobotRelativeSpeeds(drive.pose, state))

        done = drive.pose.translation
            .getDistance(target.get().translation) <= DriveConstants.AUTO_ALIGN_POS_MAX_OFFSET.`in`(Meters) &&
            Math.abs(drive.pose.rotation.minus(target.get().rotation).degrees) <= DriveConstants.AUTO_ALIGN_ANGLE_MAX_OFFSET.`in`(Radians)

        // led.alignInPos = done;
        Logger.recordOutput("Subsystems/Swerve/Align/Fine tune/good", done)
        Logger.recordOutput(
            "Subsystems/Swerve/Align/Fine tune/distance to target",
            drive.pose.translation.getDistance(target.get().translation)
        )
        Logger.recordOutput(
            "Subsystems/Swerve/Align/Fine tune/angle to target",
            Math.abs(drive.pose.rotation.minus(target.get().rotation).degrees)
        )
        Logger.recordOutput("Subsystems/Swerve/Align/Fine tune/good", false)
        Logger.recordOutput("Subsystems/Swerve/Align/Fine tune/target", target.get())
    }

    override fun end(interrupted: Boolean) {
        // led.aligningReef = false;
    }

    override fun isFinished(): Boolean {
        // end when within tolerance
        return done
    }
}
