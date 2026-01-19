package frc.robot.subsystems.turret;

import static frc.robot.constants.TurretConstants.TURRET_ANGLE_LIM;
import static frc.robot.constants.TurretConstants.TURRET_LOCK_POS;
import static frc.robot.constants.TurretConstants.TURRET_OFFSET;
import static frc.robot.constants.TurretConstants.TURRET_THETA_COMP_FACTOR;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    public double timeOfFlight = 0.0;

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
                double angle = getAngleToPos(trackpos, 
                    drive.getPose().getTranslation()//drive pos
                        .plus(new Translation2d(//turret offest
                            Math.cos(drive.getRotation().getRadians())*TURRET_OFFSET.getX(),
                            Math.sin(drive.getRotation().getRadians())*TURRET_OFFSET.getX()))
                        .plus(new Translation2d(//lead shot
                            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation()).vxMetersPerSecond,
                            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation()).vyMetersPerSecond
                        ).times(timeOfFlight))
                    ).getRadians();
                
                double offset = new Rotation2d(angle)
                    .minus(new Rotation2d(in.filteredAngle)
                    .plus(drive.getPose().getRotation())).getRadians();

                Logger.recordOutput("Turret/angle offset", offset%(2*Math.PI));
                angle = in.filteredAngle + offset;
                Logger.recordOutput("Turret/angle targeted", angle);
                if(Math.abs(angle) > TURRET_ANGLE_LIM){
                    unwinding = true;
                } else {
                    if(!unwinding){
                        io.setGoal(angle + TURRET_THETA_COMP_FACTOR*drive.getAngulerVelocity());
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

    public double getAngle(){
        return in.filteredAngle;
    }
}
