package frc.robot.subsystems.physButtons

import org.littletonrobotics.junction.AutoLog

interface ButtonIO {
    fun updateInputs(inputs: ButtonIOInputs) {}

    @AutoLog
    open class ButtonIOInputs {
        @JvmField var pressed: Boolean = false
    }
}
