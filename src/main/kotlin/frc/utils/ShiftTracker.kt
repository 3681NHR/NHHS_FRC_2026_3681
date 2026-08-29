package frc.utils

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj.Timer
import org.ironmaple.simulation.seasonspecific.rebuilt2026.Arena2026Rebuilt
import org.littletonrobotics.junction.Logger

/**
 * Ultility class that keeps track of who can score when and for how long (dependent on who won auto)
 */
object ShiftTracker {
    @JvmStatic
    private var phase: MatchPhase? = MatchPhase.UNKNOWN
    private val matchTimer: Timer = Timer()

    private var ourAlliance: Alliance? = null
    private var allianceThatWonAuto: Alliance? = null

    /* This constant stands for the amount of time that you can shoot before it's legal
     * And still have the fuel count in the hub
    */
    private const val SCORING_DELAY_SECONDS: Double = 0.5

    // Just in case we start teleop without auto and we need to correctly offset the time
    private var timeOffset: Double = 0.0

    @JvmStatic
    fun start() {
        reset()

        if (DriverStation.isAutonomous())
            phase = MatchPhase.AUTO
        else {
            phase = MatchPhase.TRANSITION_SHIFT

            val autoDuration = (MatchPhase.AUTO.getEndTime() - MatchPhase.AUTO.getStartTime()).toDouble()
            val autoToTeleopDuration =
                (MatchPhase.AUTO_TELE_TRANSITION.getEndTime() - MatchPhase.AUTO_TELE_TRANSITION.getStartTime()).toDouble()
            timeOffset = autoDuration + autoToTeleopDuration
        }

        matchTimer.start()
    }

    @JvmStatic
    fun reset() {
        matchTimer.reset()
        matchTimer.stop()

        timeOffset = 0.0
    }

    @JvmStatic
    fun getCurrentMatchPhase(): MatchPhase? = phase

    /**
     * Called periodically to ensure we accurately track which shift we are in
     */
    @JvmStatic
    fun update() {
        if (allianceThatWonAuto == null) {
            val speculatedAutoWinner = DriverStation.getGameSpecificMessage()
            if (speculatedAutoWinner.isNotEmpty())
                allianceThatWonAuto = if (speculatedAutoWinner == "R") Alliance.Red else Alliance.Blue
        }

        if (ourAlliance == null)
            ourAlliance = DriverStation.getAlliance().orElse(null)

        if (isRunning()) {
            if (phase != MatchPhase.UNKNOWN && phase != null) {
                if (getTime() > phase!!.getEndTime())
                    phase = phase!!.getNext()
            } else {
                /* Phase becomes null when we are in auto or teleop DS modes and have exceeded the standard
                 * match duration. We stop tracking shifts and any calls to canScore() will return true for testing purposes
                 */
                reset()
                phase = MatchPhase.UNKNOWN
            }

            if (RobotBase.isSimulation()) {
                // activate both hubs in sim when they should be
                (Arena2026Rebuilt.getInstance() as Arena2026Rebuilt).setShouldRunClock(
                    !(
                        phase == MatchPhase.UNKNOWN
                            || phase == MatchPhase.AUTO
                            || phase == MatchPhase.AUTO_TELE_TRANSITION
                            || phase == MatchPhase.TRANSITION_SHIFT
                            || phase == MatchPhase.ENDGAME
                        )
                )
            }
        }

        Logger.recordOutput("shift/who won auto", allianceThatWonAuto)
        Logger.recordOutput("shift/we won auto", weWonAuto())
        Logger.recordOutput("shift/can score", canScore())
        Logger.recordOutput("shift/time left in shift", getTimeLeftInShift())
        Logger.recordOutput("shift/current phase", getCurrentMatchPhase())
    }

    /**
     * @return Whether or not we can score into our alliance's hub
     */
    @JvmStatic
    fun canScore(): Boolean {
        if (!matchTimer.isRunning) return true
        else if (ourAlliance == null) return false

        return when (phase!!) {
            MatchPhase.AUTO, MatchPhase.ENDGAME, MatchPhase.UNKNOWN, MatchPhase.AUTO_TELE_TRANSITION -> true

            MatchPhase.TRANSITION_SHIFT -> {
                if (weWonAuto()) getTimeLeftInShift() >= SCORING_DELAY_SECONDS else true
            }

            MatchPhase.SHIFT1, MatchPhase.SHIFT3 -> {
                if (weWonAuto()) getTimeLeftInShift() <= SCORING_DELAY_SECONDS else getTimeLeftInShift() >= SCORING_DELAY_SECONDS
            }

            MatchPhase.SHIFT2 -> {
                if (weWonAuto()) getTimeLeftInShift() >= SCORING_DELAY_SECONDS else getTimeLeftInShift() <= SCORING_DELAY_SECONDS
            }

            MatchPhase.SHIFT4 -> {
                if (!weWonAuto()) getTimeLeftInShift() <= SCORING_DELAY_SECONDS else true
            }
        }
    }

    @JvmStatic
    fun isRunning(): Boolean = matchTimer.isRunning

    private fun weWonAuto(): Boolean = ourAlliance == allianceThatWonAuto

    @JvmStatic
    fun getTimeLeftInShift(): Double {
        return Math.max(0.0, phase!!.getEndTime() - getTime())
    }

    private fun getTime(): Double = matchTimer.get() + timeOffset
}
