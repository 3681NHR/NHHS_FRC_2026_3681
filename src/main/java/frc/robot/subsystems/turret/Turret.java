package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.constants.TurretConstants.TURRET_ANGLE_LIM;
import static frc.robot.constants.TurretConstants.TURRET_OFFSET;
import static frc.robot.constants.TurretConstants.TURRET_SYSID_CONFIG;
import static frc.robot.constants.TurretConstants.TURRET_THETA_COMP_FACTOR;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.launchLUT;
import frc.robot.subsystems.swerve.Drive;
import frc.utils.Alert;
import frc.utils.Alert.AlertType;

public class Turret extends SubsystemBase {
    
    private boolean ready = false;

    private boolean unwinding = false;
    private double unwindgoal = 0.0;

    private TurretIO io;
    private TurretIOInputsAutoLogged in = new TurretIOInputsAutoLogged();

    private Drive drive;

    private SysIdRoutine sysid = new SysIdRoutine(TURRET_SYSID_CONFIG, new SysIdRoutine.Mechanism(v -> io.setVout(v.in(Volts)), null, this));

    private Alert illegalTarg = new Alert("illegal or invalid Turret setpoint!", AlertType.kWarning);

    private Alert runningSysid = new Alert("Turret sysid running", AlertType.kInfo);

    public Turret(TurretIO io, Drive drive){
        this.io = io;
        this.drive = drive;

        Logger.recordOutput("Turret/track/target pos", (Translation2d)null);
        Logger.recordOutput("Turret/track/angle offset", Double.NaN);
        Logger.recordOutput("Turret/track/angle targeted", Double.NaN);
        Logger.recordOutput("Turret/manual/target", Double.NaN);
    }

    @Override
    public void periodic(){
        io.updateInputs(in);
        Logger.processInputs("TurretIO", in);

        Logger.recordOutput("Turret/state", (getCurrentCommand() == null ? "none" :getCurrentCommand().getName()));
        Logger.recordOutput("Turret/ready", ready);
        Logger.recordOutput("Turret/unwind angle", unwindgoal);
        Logger.recordOutput("Turret/unwinding", unwinding);

    }

    public Command manPos(DoubleSupplier targ){
        return Commands.run(() -> {
            if(Math.abs(targ.getAsDouble()) <= TURRET_ANGLE_LIM){
                io.setGoal(targ.getAsDouble());
                ready = in.atSetpoint;
            } else {
                ready = false;
            }
            illegalTarg.set(Math.abs(targ.getAsDouble()) > TURRET_ANGLE_LIM || !Double.isFinite(targ.getAsDouble()));
            Logger.recordOutput("Turret/manual/target", targ.getAsDouble());
        }, this).finallyDo(() -> {
            Logger.recordOutput("Turret/manual/target", Double.NaN);
        }).withName("manual angle");
    }

    public Command track(Supplier<Translation2d> targ){
        return Commands.run(() -> {
            double timeOfFlight = launchLUT.get(targ.get().getDistance(getFieldPos()), true, launchLUT.LUTHub)[2];
            Logger.recordOutput("Turret/track/target pos", targ.get());
                double angle = getAngleToPos(targ.get(), 
                    drive.getPose().getTranslation()//drive pos
                        .plus(new Translation2d(//turret offest
                            Math.cos(drive.getRotation().getRadians())*TURRET_OFFSET.getX(),
                            Math.sin(drive.getRotation().getRadians())*TURRET_OFFSET.getX()))
                        .plus(new Translation2d(//lead shot
                            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation()).vxMetersPerSecond,
                            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation()).vyMetersPerSecond
                        ).times(timeOfFlight))//TODO: recalculate tof at lead position, iterate n times to estimate correct aim
                    ).getRadians();
                
                double offset = new Rotation2d(angle)
                    .minus(new Rotation2d(in.filteredAngle)
                    .plus(drive.getPose().getRotation())).getRadians();

                Logger.recordOutput("Turret/track/angle offset", offset);
                angle = in.filteredAngle + offset;
                Logger.recordOutput("Turret/track/angle targeted", angle);
                if(Math.abs(angle) > TURRET_ANGLE_LIM){
                    unwinding = true;
                } else {
                    if(!unwinding){
                        io.setGoal(angle + TURRET_THETA_COMP_FACTOR*drive.getAngulerVelocity());
                        ready = in.atSetpoint;
                    } else {
                        ready = false;
                        io.setGoal(angle%TURRET_ANGLE_LIM);
                        if(in.atSetpoint){
                            unwinding = false;
                        }
                    }
                }
        }, this).finallyDo(() -> {
            Logger.recordOutput("Turret/track/target pos", (Translation2d)null);
                Logger.recordOutput("Turret/track/angle offset", Double.NaN);
                Logger.recordOutput("Turret/track/angle targeted", Double.NaN);
        }).withName("track position");
    }

    public Command sysidQuasistatic(boolean reverse){
        return sysid.quasistatic(reverse ? SysIdRoutine.Direction.kReverse : SysIdRoutine.Direction.kForward)
        .until( () -> Math.abs(in.filteredAngle) > TURRET_ANGLE_LIM)
        .raceWith(Commands.run(() -> {
            ready = false;
            runningSysid.set(true);
            runningSysid.setText("Turret sysid running: dynamic: " + (reverse ? "reverse" : "forward"));
        }))
        .withName("quasistatic sysid: " + (reverse ? "reverse" : "forward"));
    }

    public Command sysidDynamic(boolean reverse){
        return sysid.dynamic(reverse ? SysIdRoutine.Direction.kReverse : SysIdRoutine.Direction.kForward)
        .until( () -> Math.abs(in.filteredAngle) > TURRET_ANGLE_LIM)
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

    private Rotation2d getAngleToPos(Translation2d target, Translation2d curr){
        return new Rotation2d(Math.atan2(target.getY()-curr.getY(), target.getX()-curr.getX()));
    }

    public double getAngle(){
        return in.filteredAngle;
    }

    public Translation2d getFieldPos(){
        return drive.getPose().getTranslation()//drive pos
            .plus(new Translation2d(//turret offest
                Math.cos(drive.getRotation().getRadians())*TURRET_OFFSET.getX(),
                Math.sin(drive.getRotation().getRadians())*TURRET_OFFSET.getX()));
    }

}
