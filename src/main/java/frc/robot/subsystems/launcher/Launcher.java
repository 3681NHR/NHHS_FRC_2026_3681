package frc.robot.subsystems.launcher;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.Alert;
import frc.utils.Alert.AlertType;

import static frc.robot.constants.LauncherConstants.*;

public class Launcher extends SubsystemBase {
    
    LauncherIO io;
    LauncherIOInputsAutoLogged in = new LauncherIOInputsAutoLogged();

    boolean ready = false;

    private SysIdRoutine sysid = new SysIdRoutine(LAUNCHER_SYSID_CONFIG, new SysIdRoutine.Mechanism(v -> io.setVout(v), null, this));
    private Alert runningSysid = new Alert("Launcher sysid running", AlertType.kInfo);

    public Launcher(LauncherIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(in);
        Logger.processInputs("Launcher", in);
        Logger.recordOutput("Launcher/state", (getCurrentCommand() == null ? "none" : getCurrentCommand().getName()));

    }

    public Command velocityControl(Supplier<AngularVelocity> vel){
        return Commands.run(() -> {
            io.setGoal(vel.get());
        }, this)
        .withName("velocity control");
    }

    public Command voltageControl(Supplier<Voltage> volt){
        return Commands.run(() -> {
            io.setVout(volt.get());
        }, this)
        .withName("voltage control");
    }
    
    public Command sysidQuasistatic(boolean reverse){
        return sysid.quasistatic(reverse ? SysIdRoutine.Direction.kReverse : SysIdRoutine.Direction.kForward)
        .raceWith(Commands.run(() -> {
            ready = false;
            runningSysid.set(true);
            runningSysid.setText("Turret sysid running: dynamic: " + (reverse ? "reverse" : "forward"));
        }))
        .withName("quasistatic sysid: " + (reverse ? "reverse" : "forward"));
    }

    public Command sysidDynamic(boolean reverse){
        return sysid.dynamic(reverse ? SysIdRoutine.Direction.kReverse : SysIdRoutine.Direction.kForward)
        .raceWith(Commands.run(() -> {
            ready = false;
            runningSysid.set(true);
            runningSysid.setText("Turret sysid running: quasistatic: " + (reverse ? "reverse" : "forward"));
        }))
        .withName("quasistatic sysid: " + (reverse ? "reverse" : "forward"));
    }

    public boolean isReady(){
        return ready;
    }
    public AngularVelocity getSpeed(){
        return in.filteredSpeed;
    }
}
