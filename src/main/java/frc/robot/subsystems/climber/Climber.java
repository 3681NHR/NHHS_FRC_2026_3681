package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.Meters;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ClimbConstants;

public class Climber extends SubsystemBase {
    
    private final ClimberIO io;
    private final ClimberIOInputsAutoLogged in = new ClimberIOInputsAutoLogged();

    public Climber(ClimberIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(in);
        Logger.processInputs("IO/Climber", in);
        Logger.recordOutput("Subsystems/Climber/state", (getCurrentCommand() == null ? "none" : getCurrentCommand().getName()));
    }

    public Command voltageControl(Supplier<Voltage> volts) {
        return run(() -> io.setVoltage(volts));
    }

    public Command positionControl(Supplier<Distance> pos) {
        return run(() -> io.setGoal(pos));
    }
    
    public Command zeroEncoder() {
        return runOnce(() -> io.zeroEncoder());
    }

    
    /**
     * reset angle to min value and set homed to true
     * @return
     */
    public Command forceHome(){
        return new InstantCommand(() -> {
            io.setHomed(true);
            io.setPos(CLIMBER_MIN_POS);
        }, this).withName("force home");
    }

    /**
     * uses voltage commands to auto home
     * @return
     */
    public Command home(){

        Command c = new InstantCommand(() -> {
            io.setHomed(false);
        }).andThen(new HomeCommand(
            CLIMBER_HOME_VOLTAGE, 
            CLIMBER_HOME_STOP_TIME, 
            () -> HOOD_HOME_STOP_THRESH.gte(in.velocity),
            v -> io.setVout(v),
            () -> {
                io.setPos(HOOD_MIN_ANGLE);
                io.setHomed(true);
                io.setGoal(HOOD_MIN_ANGLE);
            }));
        c.addRequirements(this);
        c.setName("auto home");
        return c;
    }
}
