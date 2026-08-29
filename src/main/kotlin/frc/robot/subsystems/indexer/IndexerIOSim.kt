package frc.robot.subsystems.indexer

import edu.wpi.first.units.Units.RPM
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Voltage

class IndexerIOSim : IndexerIO {

    private var vout: Voltage = Volts.zero()

    override fun updateInputs(input: IndexerIO.IndexerIOInputs) {
        // TODO, kv is from recalc, test on bot
        input.speed = RPM.of(198.52 * vout.`in`(Volts))

        input.motorVoltageOut = vout

        input.motorConnected = true
    }

    override fun setVout(vout: Voltage) {
        this.vout = vout
    }
}
