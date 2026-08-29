package frc.utils.LEDAnim

import edu.wpi.first.wpilibj.util.Color

/**
 * Rainbow animation
 * @param length length of LED strip
 */
class RainbowAnim(length: Int) : LEDAnim(length) {
    init {
        for (i in leds.indices) {
            val hue = (((i * 180.0) / leds.size) % 180.0).toInt()
            leds[i] = Color.fromHSV(hue, 255, 255)
        }
        initLeds = leds.clone()
    }
}
