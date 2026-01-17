package frc.robot.subsystems.turret;

import static frc.robot.constants.TurretConstants.BLUE_HUB;
import static frc.robot.constants.TurretConstants.RED_HUB;
import static frc.robot.constants.TurretConstants.TURRET_ANGLE_LIM;
import static frc.robot.constants.TurretConstants.TURRET_LOCK_POS;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Drive;

public class Turret extends SubsystemBase {
    
    public enum TurretState{
        SYS_ID,
        TRACK_POS,
        LOCK,
        POSITION_MANUAL
    }

    TurretState state = TurretState.TRACK_POS;
    TurretState oldState = TurretState.LOCK;

    boolean ready = false;
    boolean unwinding = false;

    double unwindgoal = 0.0;

    TurretIO io;
    TurretIOInputsAutoLogged in = new TurretIOInputsAutoLogged();

    Drive drive;

    public Translation2d trackpos = new Translation2d();

    public Turret(TurretIO io, Drive drive){
        this.io = io;
        this.drive = drive;
    }

    @Override
    public void periodic(){
        io.updateInputs(in);
        Logger.processInputs("TurretIO", in);
        ready = in.atSetpoint;
        Logger.recordOutput("Turret/state", state);
        Logger.recordOutput("Turret/old state", oldState);
        Logger.recordOutput("Turret/ready", ready);
        Logger.recordOutput("Turret/unwind angle", unwindgoal);
        Logger.recordOutput("Turret/unwinding", unwinding);

        switch(state){
            case LOCK:
            //preset angle
                io.setGoal(TURRET_LOCK_POS);
            break;
            case POSITION_MANUAL:
            //manual angle
            break;
            case SYS_ID:
                ready = false;
            break;
            case TRACK_POS:
                Logger.recordOutput("Turret/target pos", trackpos);
                double angle = getAngleToPos(trackpos, drive.getPose().getTranslation()).getRadians();
                angle = new Rotation2d(angle).minus(new Rotation2d(in.filteredAngle)).getRadians();
                Logger.recordOutput("Turret/angle offset", angle%(2*Math.PI));
                angle = in.filteredAngle + angle;
                Logger.recordOutput("Turret/angle targeted", angle);
                if(Math.abs(angle) > TURRET_ANGLE_LIM){
                    unwinding = true;
                } else {
                    if(!unwinding){
                        io.setGoal(angle);
                    } else {
                        unwindgoal = angle;
                    }
                }
            break;
            default:
                ready = false;
            break;
        }

        if(unwinding){
            ready = false;
            io.setGoal(unwindgoal%TURRET_ANGLE_LIM);
            if(in.atSetpoint){
                unwinding = false;
            }
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
        return new Rotation2d(Math.atan2(target.getY()-curr.getY(), target.getX()-curr.getX()));
    }
}
