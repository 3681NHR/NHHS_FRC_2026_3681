package frc.utils.rumble

enum class RumblePreset {
    /**
     * single pulse
     */
    TAP,
    /**
     * two taps
     */
    DOUBLE_TAP,
    /**
     * similar to a tap, but longer
     */
    RING;

    fun load(): Array<Rumble> {
        return when (this) {
            TAP -> arrayOf(Rumble(0.2, 1.0))
            RING -> arrayOf(Rumble(0.5, 0.75))
            DOUBLE_TAP -> arrayOf(
                Rumble(0.2, 1.0),
                Rumble(0.1, 0.0),
                Rumble(0.2, 1.0)
            )
        }
    }
}
