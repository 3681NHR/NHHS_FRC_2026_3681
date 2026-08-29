package frc.robot.subsystems.kicker

import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage
import frc.robot.constants.KickerConstants.KICKER_PRELOAD_STOP_DISTANCE

class KickerIOSim : KickerIO {

    private var vout: Voltage = Volts.zero()

    override fun updateInputs(input: KickerIO.KickerIOInputs) {
        // TODO, kv is from recalc, test on bot
        input.speed = RPM.of(198.52 * vout.`in`(Volts))

        input.motorVoltageOut = vout

        input.distance = Inches.of(0.0)
        input.hasBall = input.distance.lte(KICKER_PRELOAD_STOP_DISTANCE)
        input.motorConnected = true
    }

    override fun setVout(vout: Voltage) {
        this.vout = vout
    }
}
