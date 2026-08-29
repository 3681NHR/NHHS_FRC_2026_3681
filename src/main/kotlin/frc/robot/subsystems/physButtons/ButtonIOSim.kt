package frc.robot.subsystems.physButtons

import java.util.function.BooleanSupplier

class ButtonIOSim(private val io: BooleanSupplier) : ButtonIO {

    override fun updateInputs(inputs: ButtonIO.ButtonIOInputs) {
        inputs.pressed = io.asBoolean
    }
}
