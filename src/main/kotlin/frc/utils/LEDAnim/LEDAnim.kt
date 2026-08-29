package frc.utils.LEDAnim

import edu.wpi.first.wpilibj.util.Color
import frc.robot.constants.Constants
import frc.utils.ExtraMath

open class LEDAnim {
    @JvmField var leds: Array<Color>
    @JvmField var initLeds: Array<Color>? = null

    private var scroll: Double = 0.0
    private var scrollShift: Double = 0.0

    constructor(length: Int) {
        leds = Array(length) { Color.kBlack }
    }

    constructor(length: Int, c: Color) {
        leds = Array(length) { c }
    }

    constructor(c: Array<Color>) {
        leds = c
        initLeds = c.clone()
    }

    fun getLEDs(): Array<Color> = leds

    fun update() {
        if (scroll != 0.0) {
            scrollShift += scroll * Constants.EVENT_LOOP_TIME
            leds = offset(LEDAnim(initLeds!!.clone()), scrollShift, true).getLEDs()
        }
    }

    /**
     * scroll at relative speed
     * @param speed leds per second
     */
    fun scroll(speed: Double): LEDAnim {
        scroll = speed
        return this
    }

    companion object {
        /**
         * multiplies rgb values of base and mask
         */
        @JvmStatic
        fun mask(base: LEDAnim, mask: LEDAnim): LEDAnim {
            val result = Array(base.leds.size) { i ->
                val r = base.leds[i].red * mask.leds[i].red
                val g = base.leds[i].green * mask.leds[i].green
                val b = base.leds[i].blue * mask.leds[i].blue
                Color(r, g, b)
            }
            return LEDAnim(result)
        }

        @JvmStatic
        fun offset(`in`: LEDAnim, offset: Double, wrap: Boolean): LEDAnim {
            val result = Array(`in`.leds.size) { i ->
                if (wrap) {
                    val a = (i + offset) % `in`.leds.size
                    ExtraMath.colLerp(
                        `in`.leds[a.toInt()],
                        `in`.leds[(a + 1).toInt() % `in`.leds.size],
                        a - a.toInt()
                    )
                } else {
                    if (offset < 0 || offset >= `in`.leds.size) {
                        Color.kBlack
                    } else {
                        ExtraMath.colLerp(
                            `in`.leds[(i - offset).toInt()],
                            `in`.leds[((i - offset + 1).toInt()) % `in`.leds.size],
                            (i - offset) - (i - offset).toInt()
                        )
                    }
                }
            }
            return LEDAnim(result)
        }
    }
}
