package frc.robot.subsystems.physButtons

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj.DigitalInput

class ButtonIODIO(channel: Int) : ButtonIO {

    private val dio = DigitalInput(channel)
    private val debouncer = Debouncer(0.05, Debouncer.DebounceType.kBoth)

    override fun updateInputs(inputs: ButtonIO.ButtonIOInputs) {
        inputs.pressed = debouncer.calculate(dio.get())
    }
}
