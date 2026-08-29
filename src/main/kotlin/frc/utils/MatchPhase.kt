package frc.utils

enum class MatchPhase(
    private val startTimeSeconds: Int,
    private val endTimeSeconds: Int,
    private val nextPhase: MatchPhase?
) {
    UNKNOWN(0, 0, null),
    ENDGAME(133, 163, UNKNOWN),
    SHIFT4(108, 133, ENDGAME),
    SHIFT3(83, 108, SHIFT4),
    SHIFT2(58, 83, SHIFT3),
    SHIFT1(33, 58, SHIFT2),
    TRANSITION_SHIFT(23, 33, SHIFT1),
    AUTO_TELE_TRANSITION(20, 23, TRANSITION_SHIFT),
    AUTO(0, 20, AUTO_TELE_TRANSITION);

    fun getNext(): MatchPhase? = nextPhase

    fun getStartTime(): Int = startTimeSeconds

    fun getEndTime(): Int = endTimeSeconds
}
