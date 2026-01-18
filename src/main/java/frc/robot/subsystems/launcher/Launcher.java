package frc.robot.subsystems.launcher;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase {
    
    public enum LauncherState {
        SHOOTING,
        IDLE
    }

    LauncherState state = LauncherState.SHOOTING;
    LauncherState oldState = LauncherState.IDLE;
    
    LauncherIO io;
    LauncherIOInputsAutoLogged in = new LauncherIOInputsAutoLogged();

    public Launcher() {

    }

    @Override
    public void periodic() {
        io.updateInputs(in);
        Logger.processInputs("Launcher", in);
        Logger.recordOutput("Launcher/state", state);
        Logger.recordOutput("Launcher/oldState", oldState);

        switch (state) {
            case SHOOTING:
                break;
            case IDLE:
                break;
            default:
                break;
        }
    }

    public void setState(LauncherState wanted){
        oldState = state;
        state = wanted;
    }
}
