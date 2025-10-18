package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.swerve.Drive;
import frc.robot.subsystems.vision.Vision;
import frc.utils.VariableLimSLR;

public class Superstructure extends SubsystemBase {
    public enum WantedSuperState {
        DEFAULT_STATE,
        STOPPED
    }

    public enum CurrentSuperState {
        NO_PIECE,
        HOLDING_PIECE,
        STOPPED
    }

    public WantedSuperState wantedState = WantedSuperState.DEFAULT_STATE;
    public CurrentSuperState currentState = CurrentSuperState.STOPPED;
    public CurrentSuperState previousState = CurrentSuperState.STOPPED;

    Drive drive;
    Vision vision;
    Led led;

    @AutoLogOutput(key = "Superstructure/is scoring")
    public boolean scoring = false;

    private VariableLimSLR lxLim;
    private VariableLimSLR lyLim;
    private VariableLimSLR rxLim;
    private VariableLimSLR ryLim;

    private boolean fod = Constants.drive.STARTING_FOD;

    private LoggedNetworkBoolean useVisionOdometry = new LoggedNetworkBoolean("overrides/useVisionOdometry",
            DriveConstants.USE_VISION);

    public Superstructure(
            Drive drive,
            Vision vision,
            Led led,
            VariableLimSLR lxLim,
            VariableLimSLR lyLim,
            VariableLimSLR rxLim,
            VariableLimSLR ryLim) {
        this.drive = drive;
        this.vision = vision;
        this.led = led;
        this.lxLim = lxLim;
        this.lyLim = lyLim;
        this.rxLim = rxLim;
        this.ryLim = ryLim;

    }

    @Override
    public void periodic() {
        previousState = currentState;

        // rate limiter lowers max commanded accel to prevent tipping
        double rlim = Double.POSITIVE_INFINITY;
        lxLim.setLim(rlim);
        lyLim.setLim(rlim);
        rxLim.setLim(rlim);
        ryLim.setLim(rlim);

        drive.setFOD(fod);

        Logger.recordOutput("Drive/fieldOrientedDrive", getFOD());

        DriveConstants.USE_VISION = useVisionOdometry.get();

        updateAScopePoses();

        Logger.recordOutput("Superstructure/previousState", previousState);
        Logger.recordOutput("Superstructure/currentState", currentState);
        Logger.recordOutput("Superstructure/wantedState", wantedState);

        stateTransition();
        applyStates();

    }

    // update current state
    private void stateTransition() {
        switch (wantedState) {
            case STOPPED:
                currentState = CurrentSuperState.STOPPED;
                break;
            case DEFAULT_STATE:

                break;
            default:
                currentState = CurrentSuperState.STOPPED;
                break;
        }
    }

    // apply current state
    private void applyStates() {

        switch (currentState) {
            case STOPPED:
                break;
            case NO_PIECE:
                break;
            case HOLDING_PIECE:
                break;
            default:
                break;
        }
    }

    public void setWantedState(WantedSuperState state) {
        wantedState = state;
    }

    public boolean getFOD() {
        return fod;
    }


    public void updateAScopePoses() {
        
    }

}
