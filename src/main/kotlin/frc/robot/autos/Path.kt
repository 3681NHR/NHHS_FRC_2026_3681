package frc.robot.autos

import com.pathplanner.lib.path.PathPlannerPath
import com.pathplanner.lib.trajectory.PathPlannerTrajectory
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.Command as WPILibCommand
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.RobotContainer
import frc.robot.constants.DriveConstants
import java.util.LinkedList
import java.util.function.Supplier

private fun getTraj(path: PathPlannerPath): PathPlannerTrajectory? {
    return (if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red) path.flipPath() else path)
        .getIdealTrajectory(DriveConstants.PP_CONFIG).orElse(null)
}

private fun getChoreoTraj(name: String): LinkedList<PathPlannerTrajectoryState> {
    return try {
        LinkedList(getTraj(PathPlannerPath.fromChoreoTrajectory(name))!!.states)
    } catch (e: Exception) {
        println("error getting path: $name")
        e.printStackTrace()
        LinkedList()
    }
}

private fun followChoreoPath(pathName: String): WPILibCommand {
    return try {
        Path.container!!.getDrive().followPath(PathPlannerPath.fromChoreoTrajectory(pathName))
    } catch (e: Exception) {
        e.printStackTrace()
        Commands.none()
    }
}

private fun addDelayToEnd(input: LinkedList<PathPlannerTrajectoryState>, delay: Double): LinkedList<PathPlannerTrajectoryState> {
    input.add(input.last.copyWithTime(input.last.timeSeconds + delay))
    return input
}

private fun addDelayToStart(input: LinkedList<PathPlannerTrajectoryState>, delay: Double): LinkedList<PathPlannerTrajectoryState> {
    for (v in input) {
        v.timeSeconds += delay
    }
    return input
}

private fun intake(): WPILibCommand = Path.container!!.intake()
private fun shoot(): WPILibCommand = Path.container!!.unload().withTimeout(0.2).andThen(Path.container!!.fire())
private fun track(): WPILibCommand = Path.container!!.getTrackCommand()
private fun lowerHood(): WPILibCommand = Path.container!!.getHood().retract()
private fun agitate(): WPILibCommand = Path.container!!.intake().withTimeout(1.0).andThen(Commands.waitSeconds(1.0)).repeatedly()

/**
 * enum containing all paths
 * the length of the trajectory should be the same as the execution time of the command for preview accuracy
 */
enum class Path(
    @JvmField val end: String,
    @JvmField var options: Array<Path>,
    @JvmField val ppPath: LinkedList<PathPlannerTrajectoryState>?,
    @JvmField val displayName: String,
    @JvmField val Command: Supplier<WPILibCommand>
) {
    L_TRENCH_TO_DEPOT(
        "Right of depot",
        arrayOf(),
        getChoreoTraj("A"),
        "path a",
        Supplier {
            Commands.parallel(
                followChoreoPath("A"),
                Commands.waitSeconds(1.0).andThen(
                    Commands.parallel(
                        track(),
                        intake()
                    )
                ),
                Commands.waitSeconds(2.7).andThen(shoot())
            )
        }
    ),
    L_BUMP_TO_DEPOT(
        "Right of depot",
        arrayOf(),
        addDelayToStart(getChoreoTraj("B"), 1.0),
        "path b",
        Supplier {
            Commands.parallel(
                followChoreoPath("B"),
                Commands.waitSeconds(1.0).andThen(
                    Commands.parallel(
                        track(),
                        intake()
                    )
                ),
                Commands.waitSeconds(2.7).andThen(shoot())
            )
        }
    ),
    MIDDLE(
        "Middle of zone",
        arrayOf(),
        getChoreoTraj("C"),
        "path c",
        Supplier {
            Commands.sequence(
                followChoreoPath("C"),
                Commands.parallel(
                    track(),
                    shoot()
                )
            )
        }
    ),
    R_BUMP_TO_OUTPOST(
        "Outpost",
        arrayOf(),
        getChoreoTraj("D"),
        "path d",
        Supplier {
            Commands.parallel(
                followChoreoPath("D"),
                Commands.waitSeconds(1.0).andThen(
                    Commands.parallel(
                        track(),
                        intake()
                    )
                ),
                Commands.waitSeconds(2.5).andThen(shoot())
            )
        }
    ),
    MIDDLE_TO_OUTPOST(
        "Outpost",
        arrayOf(),
        getChoreoTraj("E"),
        "path e",
        Supplier {
            Commands.parallel(
                followChoreoPath("E"),
                Commands.waitSeconds(1.0).andThen(
                    Commands.parallel(
                        track(),
                        intake()
                    )
                ),
                Commands.waitSeconds(2.6).andThen(shoot())
            )
        }
    ),
    R_TRENCH_TO_OUTPOST(
        "Outpost",
        arrayOf(),
        getChoreoTraj("F"),
        "path f",
        Supplier {
            Commands.parallel(
                followChoreoPath("F"),
                Commands.waitSeconds(1.0).andThen(
                    Commands.parallel(
                        track(),
                        intake()
                    )
                ),
                Commands.waitSeconds(2.5).andThen(shoot())
            )
        }
    ),
    L_TRENCH_TO_MID_PICKUP(
        "Left side of neutral zone",
        arrayOf(),
        getChoreoTraj("G"),
        "path g(intake only)",
        Supplier {
            Commands.deadline(
                followChoreoPath("G"),
                Commands.waitSeconds(1.5).andThen(intake())
            )
        }
    ),
    L_TRENCH_TO_MID_PASS(
        "Left side of neutral zone",
        arrayOf(),
        getChoreoTraj("G"),
        "path g(pass)",
        Supplier {
            Commands.deadline(
                followChoreoPath("G"),
                Commands.waitSeconds(1.0).andThen(track()),
                Commands.waitSeconds(1.5).andThen(intake()),
                Commands.waitSeconds(2.7).andThen(shoot())
            )
        }
    ),
    R_TRENCH_TO_MID_PICKUP(
        "Right side of neutral zone",
        arrayOf(),
        getChoreoTraj("H"),
        "path h(intake only)",
        Supplier {
            Commands.deadline(
                followChoreoPath("H"),
                Commands.waitSeconds(1.5).andThen(intake())
            )
        }
    ),
    R_TRENCH_TO_MID_PASS(
        "Right side of neutral zone",
        arrayOf(),
        getChoreoTraj("H"),
        "path h(pass)",
        Supplier {
            Commands.deadline(
                followChoreoPath("H"),
                Commands.waitSeconds(1.0).andThen(track()),
                Commands.waitSeconds(1.5).andThen(intake()),
                Commands.waitSeconds(2.7).andThen(shoot())
            )
        }
    ),
    L_MID_TO_L_TRENCH(
        "Left trench",
        arrayOf(),
        getChoreoTraj("I"),
        "path i",
        Supplier {
            Commands.deadline(
                followChoreoPath("I"),
                lowerHood()
            )
        }
    ),
    R_MID_TO_R_TRENCH(
        "Right trench",
        arrayOf(),
        getChoreoTraj("J"),
        "path j",
        Supplier {
            Commands.deadline(
                followChoreoPath("J"),
                lowerHood()
            )
        }
    ),
    L_MID_TO_L_BUMP(
        "Left bump",
        arrayOf(),
        getChoreoTraj("K"),
        "path k",
        Supplier { followChoreoPath("K") }
    ),
    L_MID_TO_L_BUMP_AND_FIRE(
        "Fire at left bump(5 sec)",
        arrayOf(),
        addDelayToEnd(getChoreoTraj("K"), 5.0),
        "path k(with pause)",
        Supplier {
            Commands.deadline(
                followChoreoPath("K").andThen(
                    Commands.parallel(
                        shoot(),
                        agitate()
                    ).withTimeout(5.0)
                ),
                track()
            )
        }
    ),
    R_MID_TO_R_BUMP(
        "Right bump",
        arrayOf(),
        getChoreoTraj("L"),
        "path l",
        Supplier { followChoreoPath("L") }
    ),
    R_MID_TO_R_BUMP_AND_FIRE(
        "fire at right bump(5 sec)",
        arrayOf(),
        getChoreoTraj("L"),
        "path l(with pause)",
        Supplier {
            Commands.deadline(
                followChoreoPath("L").andThen(
                    Commands.parallel(
                        shoot(),
                        agitate()
                    ).withTimeout(5.0)
                ),
                track()
            )
        }
    ),
    START_L_TRENCH("Left trench", arrayOf(), null, "", Supplier { Commands.none() }),
    START_L_BUMP("Left bump", arrayOf(), null, "", Supplier { Commands.none() }),
    START_MID("Middle start", arrayOf(), null, "", Supplier { Commands.none() }),
    START_R_BUMP("Right bump", arrayOf(), null, "", Supplier { Commands.none() }),
    START_R_TRENCH("Right trench", arrayOf(), null, "", Supplier { Commands.none() }),
    START(
        "",
        arrayOf(
            START_L_TRENCH,
            START_L_BUMP,
            START_MID,
            START_R_BUMP,
            START_R_TRENCH
        ),
        LinkedList(),
        "",
        Supplier { Commands.none() }
    );

    private fun mergeTrajectories(vararg inTrajs: LinkedList<PathPlannerTrajectoryState>): LinkedList<PathPlannerTrajectoryState> {
        val traj = LinkedList<PathPlannerTrajectoryState>()
        var timeOffset = 0.0
        for (trajectory in inTrajs) {
            val states = trajectory.toTypedArray()
            val nextOffset = if (states.isNotEmpty()) states.last().timeSeconds else 0.0
            for (s in states) {
                traj.add(s.copyWithTime(s.timeSeconds + timeOffset))
            }
            timeOffset += nextOffset
        }
        return traj
    }

    companion object {
        @JvmField
        var container: RobotContainer? = null

        init {
            START_L_TRENCH.options = arrayOf(L_TRENCH_TO_MID_PASS, L_TRENCH_TO_MID_PICKUP, L_TRENCH_TO_DEPOT)
            START_L_BUMP.options = arrayOf(L_BUMP_TO_DEPOT)
            START_MID.options = arrayOf(MIDDLE_TO_OUTPOST, MIDDLE)
            START_R_BUMP.options = arrayOf(R_BUMP_TO_OUTPOST)
            START_R_TRENCH.options = arrayOf(R_TRENCH_TO_MID_PASS, R_TRENCH_TO_MID_PICKUP, R_TRENCH_TO_OUTPOST)

            L_TRENCH_TO_DEPOT.options = arrayOf()
            L_BUMP_TO_DEPOT.options = arrayOf()
            MIDDLE.options = arrayOf()
            R_BUMP_TO_OUTPOST.options = arrayOf()
            MIDDLE_TO_OUTPOST.options = arrayOf()
            R_TRENCH_TO_OUTPOST.options = arrayOf()
            L_TRENCH_TO_MID_PASS.options = arrayOf(L_MID_TO_L_TRENCH, L_MID_TO_L_BUMP, L_MID_TO_L_BUMP_AND_FIRE)
            R_TRENCH_TO_MID_PASS.options = arrayOf(R_MID_TO_R_TRENCH, R_MID_TO_R_BUMP, R_MID_TO_R_BUMP_AND_FIRE)

            L_TRENCH_TO_MID_PICKUP.options = L_TRENCH_TO_MID_PASS.options
            R_TRENCH_TO_MID_PICKUP.options = R_TRENCH_TO_MID_PASS.options
            L_MID_TO_L_TRENCH.options = START_L_TRENCH.options
            R_MID_TO_R_TRENCH.options = START_R_TRENCH.options
            L_MID_TO_L_BUMP_AND_FIRE.options = START_L_BUMP.options
            L_MID_TO_L_BUMP.options = START_L_BUMP.options
            R_MID_TO_R_BUMP_AND_FIRE.options = START_R_BUMP.options
            R_MID_TO_R_BUMP.options = START_R_BUMP.options
        }
    }
}
