package frc.robot.subsystems.indexer;

import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public class IndexerIOSim implements IndexerIO {

    Voltage vout = Volts.zero();


    public IndexerIOSim() {
    }

    @Override
    public void updateInputs(IndexerIOInputs input) {
        //TODO, kv is from recalc, test on bot
        input.speed = RPM.of(198.52*vout.in(Volts));

        input.motorVoltageOut = vout;

        input.motorConnected = true;
    }

    @Override
    public void setVout(Voltage vout) {
        this.vout = vout;
    }
}
