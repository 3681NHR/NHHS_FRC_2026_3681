package frc.utils

import edu.wpi.first.math.MathSharedStore
import edu.wpi.first.math.MathUtil

/**
 * A class that limits the rate of change of an input value. Useful for implementing voltage,
 * setpoint, and/or output ramps. A slew-rate limit is most appropriate when the quantity being
 * controlled is a velocity or a voltage; when controlling a position, consider using a
 * [edu.wpi.first.math.trajectory.TrapezoidProfile] instead.
 */
class VariableLimSLR {
    private var m_positiveRateLimit: Double
    private var m_negativeRateLimit: Double
    private var m_prevVal: Double
    private var m_prevTime: Double

    /**
     * Creates a new SlewRateLimiter with the given positive and negative rate limits and initial
     * value.
     *
     * @param positiveRateLimit The rate-of-change limit in the positive direction, in units per
     *     second. This is expected to be positive.
     * @param negativeRateLimit The rate-of-change limit in the negative direction, in units per
     *     second. This is expected to be negative.
     * @param initialValue The initial value of the input.
     */
    constructor(positiveRateLimit: Double, negativeRateLimit: Double, initialValue: Double) {
        m_positiveRateLimit = positiveRateLimit
        m_negativeRateLimit = negativeRateLimit
        m_prevVal = initialValue
        m_prevTime = MathSharedStore.getTimestamp()
    }

    /**
     * Creates a new SlewRateLimiter with the given positive rate limit and negative rate limit of
     * -rateLimit.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    constructor(rateLimit: Double) : this(rateLimit, -rateLimit, 0.0)

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    fun calculate(input: Double): Double {
        val currentTime = MathSharedStore.getTimestamp()
        val elapsedTime = currentTime - m_prevTime
        m_prevVal += MathUtil.clamp(
            input - m_prevVal,
            m_negativeRateLimit * elapsedTime,
            m_positiveRateLimit * elapsedTime
        )
        m_prevTime = currentTime
        return m_prevVal
    }

    /**
     * Returns the value last calculated by the SlewRateLimiter.
     *
     * @return The last value.
     */
    fun lastValue(): Double = m_prevVal

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    fun reset(value: Double) {
        m_prevVal = value
        m_prevTime = MathSharedStore.getTimestamp()
    }

    fun setLim(lim: Double) {
        m_positiveRateLimit = lim
        m_negativeRateLimit = -lim
    }

    fun setLim(plim: Double, nlim: Double) {
        m_positiveRateLimit = plim
        m_negativeRateLimit = nlim
    }
}
