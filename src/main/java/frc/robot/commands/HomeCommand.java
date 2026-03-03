package frc.robot.commands;

import static edu.wpi.first.units.Units.Microseconds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * generic homing command
 */
public class HomeCommand extends Command {
    Voltage homeVolts;
    Time stopTime;
    BooleanSupplier stoppedSupplier;
    Consumer<Voltage> voltageConsumer;
    Runnable onHome;

    double stopTimestamp = Double.NaN;

    public HomeCommand(Voltage homeVolts, Time stopTime, BooleanSupplier stoppedSupplier, Consumer<Voltage> voltageConsumer, Runnable onHome) {
        this.homeVolts = homeVolts;
        this.stopTime = stopTime;
        this.stoppedSupplier = stoppedSupplier;
        this.voltageConsumer = voltageConsumer;
        this.onHome = onHome;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        voltageConsumer.accept(homeVolts);

        if(!stoppedSupplier.getAsBoolean()){
            stopTimestamp = Double.NaN;
        } else if(Double.isNaN(stopTimestamp)){
            stopTimestamp = Logger.getTimestamp();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(!interrupted){
            onHome.run();
        }
        voltageConsumer.accept(Volts.of(0));
    }

    @Override
    public boolean isFinished() {
        if(Double.isNaN(stopTimestamp)){
            return false;
        }
        if(Microseconds.of(Logger.getTimestamp()-stopTimestamp).gte(stopTime)){
            stopTimestamp = Double.NaN;
        }
        return Microseconds.of(Logger.getTimestamp()-stopTimestamp).gte(stopTime);
    }
}
