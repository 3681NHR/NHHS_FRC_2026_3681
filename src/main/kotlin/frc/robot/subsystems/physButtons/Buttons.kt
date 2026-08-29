package frc.robot.subsystems.physButtons

import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.littletonrobotics.junction.Logger

class Buttons(private vararg val ios: ButtonIO) : SubsystemBase() {

    private val inputs: Array<ButtonIOInputsAutoLogged?> = arrayOfNulls(ios.size)

    override fun periodic() {
        for (i in ios.indices) {
            if (inputs[i] == null) {
                inputs[i] = ButtonIOInputsAutoLogged()
            }
            ios[i].updateInputs(inputs[i]!!)
            Logger.processInputs("IO/Buttons/$i", inputs[i])
        }
    }

    operator fun get(index: Int): Boolean {
        if (index < inputs.size) {
            if (inputs[index] == null) {
                inputs[index] = ButtonIOInputsAutoLogged()
            }
            return inputs[index]!!.pressed
        }
        return false
    }
}
