package frc.robot.subsystems.kicker;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.utils.controlWrappers.SimpleFF;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.KickerConstants.*;

public class KickerIOSim implements KickerIO {

    private final SimpleFF FF = new SimpleFF(KICKER_FF_GAINS);

    LinearVelocity goal = MetersPerSecond.zero();
    Voltage vout = Volts.zero();

    private boolean openLoop = false;

    public KickerIOSim() {
        KICKER_FF_GAINS.withCallback(() -> {
            FF.setGains(KICKER_FF_GAINS);
        });
    }

    @Override
    public void updateInputs(KickerIOInputs input) {
        
        if (!openLoop) {
            vout = Volts.of(FF.calculate(input.goal.in(MetersPerSecond)));
        }

        input.speed = goal;
        input.goal = goal;

        input.motorVoltageOut = vout;

        input.openLoop = openLoop;

        input.distance = Inches.of(0);
        input.hasBall = input.distance.lte(KICKER_PRELOAD_STOP_DISTANCE);
        input.motorConnected = true;
    }

    @Override
    public void setVout(Voltage vout) {
        this.openLoop = true;
        this.vout = vout;
    }

    @Override
    public void setGoal(LinearVelocity goal) {
        this.openLoop = false;
        this.goal = goal;
    }
}
