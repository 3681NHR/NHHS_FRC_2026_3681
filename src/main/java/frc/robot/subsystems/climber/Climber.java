package frc.robot.subsystems.climber;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.HomeCommand;
import frc.utils.HiddenConditionalCommand;

import static frc.robot.constants.ClimberConstants.*;

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
        return run(() -> io.setVoltage(volts.get()));
    }

    public Command positionControl(Supplier<Distance> pos) {
        return run(() -> io.setGoal(pos.get()));
    }
    public Distance getPosition(){
        return in.position;
    }
    
    /**
     * reset position to min value and set homed to true
     * @return
     */
    public Command forceHome(){
        return new InstantCommand(() -> {
            io.setHomed(true);
            io.setPosition(CLIMBER_MIN_POSITION);
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
            () -> CLIMBER_HOME_STOP_THRESH.gte(in.velocity),
            v -> io.setVoltage(v),
            () -> {
                io.setPosition(CLIMBER_MIN_POSITION);
                io.setHomed(true);
                io.setGoal(CLIMBER_MIN_POSITION);
            }));
        c.addRequirements(this);
        c.setName("auto home");
        return c;
    }

    public Command extend(){
        return positionControl(() -> CLIMBER_MAX_POSITION);
    }
    
    public Command retract(){
        return positionControl(() -> CLIMBER_MIN_POSITION);
    }

    public Command toggle(){
        return new HiddenConditionalCommand(
            extend(),
            retract(),
            () -> in.goal.baseUnitMagnitude() <= (CLIMBER_MAX_POSITION.baseUnitMagnitude()/2.0)
        );
    }
}