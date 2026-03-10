package frc.robot.subsystems.kicker;

import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.KickerConstants.*;

public class KickerIOSim implements KickerIO {

    Voltage vout = Volts.zero();


    public KickerIOSim() {
    }

    @Override
    public void updateInputs(KickerIOInputs input) {
        //TODO, kv is from recalc, test on bot
        input.speed = RPM.of(198.52*vout.in(Volts));

        input.motorVoltageOut = vout;

        input.distance = Inches.of(0);
        input.hasBall = input.distance.lte(KICKER_PRELOAD_STOP_DISTANCE);
        input.motorConnected = true;
    }

    @Override
    public void setVout(Voltage vout) {
        this.vout = vout;
    }
}
