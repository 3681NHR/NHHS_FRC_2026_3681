package frc.utils

import edu.wpi.first.math.geometry.Translation2d
import java.util.function.DoubleSupplier

object Joystick {
    @JvmStatic
    fun deadzone(deadzone: Double, x: Double, y: Double): Translation2d {
        if (ExtraMath.getMagnitude(x, y) < deadzone) {
            return Translation2d()
        }
        return Translation2d(x, y)
    }

    class DuelJoystickAxis @JvmOverloads constructor(
        @JvmField var lx: DoubleSupplier = DoubleSupplier { 0.0 },
        @JvmField var ly: DoubleSupplier = DoubleSupplier { 0.0 },
        @JvmField var rx: DoubleSupplier = DoubleSupplier { 0.0 },
        @JvmField var ry: DoubleSupplier = DoubleSupplier { 0.0 }
    )
}
