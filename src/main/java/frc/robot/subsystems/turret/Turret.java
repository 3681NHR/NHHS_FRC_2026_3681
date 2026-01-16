package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase {
    
    public enum TurretState{
        SYS_ID,
        TRACK_HUB,
        LOCK,
        TRACK_PASS,
        TRACK_MANUAL,
        POSITION_MANUAL,
        UNWIND
    }

    TurretState state = TurretState.LOCK;
    TurretState oldState = TurretState.LOCK;

    boolean ready = false;

    TurretIO io;
    TurretIOInputsAutoLogged in = new TurretIOInputsAutoLogged();

    public Turret(TurretIO io){
        this.io = io;
    }

    @Override
    public void periodic(){
        io.updateInputs(in);
        Logger.processInputs("TurretIO", in);
        ready = in.atSetpoint;

        switch(state){
            case LOCK:
            //preset angle
            break;
            case POSITION_MANUAL:
            //manual angle
            break;
            case SYS_ID:
                ready = false;
            break;
            case TRACK_HUB:

            break;
            case TRACK_MANUAL:
            //track manual target
            break;
            case TRACK_PASS:
            //track nearest pass pose for alliance
            break;
            case UNWIND:
                io.setGoal(0.0);
                if(Math.abs(in.filteredAngle) < Units.degreesToRadians(15)){
                    setState(oldState);
                }
                ready = false;
            break;
            default:
                ready = false;
            break;
        }
    }
    public boolean isReady(){
        return ready;
    }

    public void setState(TurretState wanted){
        oldState = state;
        state = wanted;
    }
    
    private Rotation2d getAngleToPos(Translation2d target, Translation2d curr){
        return new Rotation2d(Math.atan2(target.getX()-curr.getX(), target.getY()-curr.getY()));
    }
}
