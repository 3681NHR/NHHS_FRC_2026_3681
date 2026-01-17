package frc.robot.subsystems.turret;

import static frc.robot.constants.TurretConstants.BLUE_HUB;
import static frc.robot.constants.TurretConstants.RED_HUB;
import static frc.robot.constants.TurretConstants.TURRET_ANGLE_LIM;
import static frc.robot.constants.TurretConstants.TURRET_LOCK_POS;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Drive;

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

    TurretState state = TurretState.TRACK_HUB;
    TurretState oldState = TurretState.LOCK;

    boolean ready = false;

    TurretIO io;
    TurretIOInputsAutoLogged in = new TurretIOInputsAutoLogged();

    Drive drive;

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
            case TRACK_HUB:
                Translation2d hub = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? BLUE_HUB : RED_HUB;
                Logger.recordOutput("Turret/hub pos", hub);
                double angle = getAngleToPos(hub, drive.getPose().getTranslation()).getRadians();
                angle = new Rotation2d(angle).minus(new Rotation2d(in.filteredAngle)).getRadians();
                Logger.recordOutput("Turret/angle offset", angle%(2*Math.PI));
                angle = in.filteredAngle + angle  - drive.getPose().getRotation().getRadians();
                Logger.recordOutput("Turret/angle targeted", angle + drive.getPose().getRotation().getRadians());
                if(Math.abs(angle) > TURRET_ANGLE_LIM){
                    setState(TurretState.UNWIND);
                } else {
                    io.setGoal(angle);
                }
            break;
            case TRACK_MANUAL:
            //track manual target
            break;
            case TRACK_PASS:
            //track nearest pass pose for alliance
            break;
            case UNWIND:
                io.setGoal(MathUtil.angleModulus(in.goal));
                if(in.atSetpoint){
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
        if(state != TurretState.UNWIND){   
            oldState = state;
        }
            state = wanted;
    }
    
    private Rotation2d getAngleToPos(Translation2d target, Translation2d curr){
        return new Rotation2d(Math.atan2(target.getY()-curr.getY(), target.getX()-curr.getX()));
    }
}
