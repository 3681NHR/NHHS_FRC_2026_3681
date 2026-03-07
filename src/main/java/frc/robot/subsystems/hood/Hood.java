package frc.robot.subsystems.hood;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.HomeCommand;
import frc.robot.subsystems.SOTMSolver;
import frc.utils.ExtraMath;

import static frc.robot.constants.HoodConstants.*;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
    
    HoodIO io;
    HoodIOInputsAutoLogged in = new HoodIOInputsAutoLogged();

    boolean homing = false;
    boolean manual = false;

    public Hood(HoodIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(in);
        Logger.processInputs("IO/Hood", in);

        Logger.recordOutput("Subsystems/Hood/state", getCurrentCommand() == null ? "none" : getCurrentCommand().getName());
        
    }

    public Command trackWithLead(){
        return positionControl(() -> SOTMSolver.getInstance().getParams(false).hoodAngle()).withName("SOTM position control");
    }

    /**
     * position control, soft limits apply, and setpoint is clamped
     * @param pos
     * @return
     */
    public Command positionControl(Supplier<Angle> pos){
        return new InstantCommand(() -> {
            manual = true;
        }).andThen(Commands.run(() -> {
            io.setGoal(ExtraMath.clamp(pos.get(), HOOD_MIN_ANGLE, HOOD_MAX_ANGLE));
        }, this)).finallyDo(() -> {
            manual = false;
        }).withName("position control");
    }

    /**
     * set openloop vout, soft limits will still apply if homed
     * @param vout - voltage to apply
     * @return
     */
    public Command voltageControl(Supplier<Voltage> vout){
        return Commands.run(() -> {
            io.setVout(vout.get());
        }, this).withName("voltage control");
    }

    /**
     * reset angle to min value and set homed to true
     * @return
     */
    public Command forceHome(){
        return new InstantCommand(() -> {
            io.setHomed(true);
            io.setPos(HOOD_MIN_ANGLE);
        }, this).withName("force home");
    }

    /**
     * uses voltage commands to auto home
     * @return
     */
    public Command home(){

        Command c = new InstantCommand(() -> {
            io.setHomed(false);
            homing = true;
        }).andThen(new HomeCommand(
            HOOD_HOME_VOLTAGE, 
            HOOD_HOME_STOP_TIME, 
            () -> HOOD_HOME_STOP_THRESH.gte(in.velocity),
            v -> io.setVout(v),
            () -> {
                io.setPos(HOOD_MIN_ANGLE);
                io.setHomed(true);
                io.setGoal(HOOD_MIN_ANGLE);
            })).andThen(new InstantCommand(() -> {
                homing = false;
            }));
        c.addRequirements(this);
        c.setName("auto home");
        return c;
    }
    public Angle getAngle(){
        return in.angle;
    }

    @AutoLogOutput(key="Subsystems/Hood/ready")
    public boolean isReady(){
        return (in.atSetpoint || in.openloop) && in.homed;
    }

    public boolean isHomed(){
        return in.homed;
    }

    public boolean isHoming(){
        return homing;
    }

    public boolean isManual(){
        return manual;
    }
}
