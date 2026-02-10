package frc.robot.subsystems.turret;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radian;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static frc.robot.constants.TurretConstants.TURRET_ANGLE_LIM;
import static frc.robot.constants.TurretConstants.TURRET_OFFSET;
import static frc.robot.constants.TurretConstants.TURRET_SYSID_CONFIG;
import static frc.robot.constants.TurretConstants.TURRET_THETA_COMP_FACTOR;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.launchLUT;
import frc.robot.subsystems.swerve.Drive;
import frc.utils.ExtraMath;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public class Turret extends SubsystemBase {
    
    private boolean ready = false;

    private boolean unwinding = false;
    private Angle unwindgoal = Radians.of(0.0);

    private TurretIO io;
    private TurretIOInputsAutoLogged in = new TurretIOInputsAutoLogged();

    private Drive drive;

    private SysIdRoutine sysid = new SysIdRoutine(TURRET_SYSID_CONFIG, new SysIdRoutine.Mechanism(v -> io.setVout(v), null, this));

    private Alert illegalTarg = new Alert("illegal or invalid Turret setpoint!", AlertType.kWarning);

    private Alert runningSysid = new Alert("Turret sysid running", AlertType.kInfo);

    public Turret(TurretIO io, Drive drive){
        this.io = io;
        this.drive = drive;

        //init log values
        Logger.recordOutput("Subsystems/Turret/track/pathing/modAngle", Double.NaN, Rotations);
        Logger.recordOutput("Subsystems/Turret/track/pathing/modCurrent", Double.NaN, Rotations);
        Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset", Double.NaN, Rotations);
        Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset A", Double.NaN, Rotations);
        Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset B", Double.NaN, Rotations);
        Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset c", Double.NaN, Rotations);
        Logger.recordOutput("Subsystems/Turret/track/angle targeted", Double.NaN, Rotations);
        Logger.recordOutput("Subsystems/Turret/track/lead time", Double.NaN, Seconds);
        Logger.recordOutput("Subsystems/Turret/track/target pos", (Translation2d)null);
        Logger.recordOutput("Subsystems/Turret/track/virtual target pos", (Translation2d)null);
    
        Logger.recordOutput("Subsystems/Turret/manual/target", Double.NaN, Rotations);
    }

    @Override
    public void periodic(){
        io.updateInputs(in);
        Logger.processInputs("IO/Turret", in);

        Logger.recordOutput("Subsystems/Turret/state", (getCurrentCommand() == null ? "none" :getCurrentCommand().getName()));
        Logger.recordOutput("Subsystems/Turret/ready", ready);
        Logger.recordOutput("Subsystems/Turret/unwind angle", unwindgoal);
        Logger.recordOutput("Subsystems/Turret/unwinding", unwinding);

        Logger.recordOutput("Subsystems/Turret/field angle", in.filteredAngle.plus(Radians.of(drive.getRotation().getRadians())).plus(DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? Degrees.of(180) : Degrees.of(0)));
    }

    public Command manPos(Supplier<Angle> targ){
        return Commands.run(() -> {
            if(targ.get().abs(Radian) <= TURRET_ANGLE_LIM.in(Radians)){
                io.setGoal(targ.get());
                ready = in.atSetpoint;
            } else {
                ready = false;
            }
            illegalTarg.set(targ.get().abs(Radian) > TURRET_ANGLE_LIM.in(Radians));
            Logger.recordOutput("Subsystems/Turret/manual/target", targ.get());
        }, this).finallyDo(() -> {
            Logger.recordOutput("Subsystems/Turret/manual/target", Double.NaN);
        }).withName("manual angle");
    }

    public Command track(Supplier<Translation2d> targ){
        return Commands.run(() -> {
            double dist = targ.get().getDistance(getFieldPos());
            Logger.recordOutput("Subsystems/Turret/track/distance", dist);
            double timeOfFlight = launchLUT.get(dist, true, launchLUT.LUTHub)[2];

            Translation2d virtualTarg = targ.get()
                        .plus(new Translation2d(//lead shot
                            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation()).vxMetersPerSecond,
                            ChassisSpeeds.fromRobotRelativeSpeeds(drive.getChassisSpeeds(), drive.getRotation()).vyMetersPerSecond
                        ).times(timeOfFlight));

            Angle angle = getAngleToPos(
                virtualTarg, //target(offset for lead)
                    drive.getPose().getTranslation()//drive pos
                        .plus(new Translation2d(//turret offest
                            Math.cos(drive.getRotation().getRadians())*TURRET_OFFSET.getX(),
                            Math.sin(drive.getRotation().getRadians())*TURRET_OFFSET.getX()))
                )
                .minus(Radians.of(drive.getPose().getRotation().getRadians()))
                .plus(Radians.of(TURRET_THETA_COMP_FACTOR*drive.getAngulerVelocity().in(RadiansPerSecond)));
                
            double modAngle = MathUtil.inputModulus(angle.in(Rotations), 0, 1);
            double modCurrent = MathUtil.inputModulus(in.goal.in(Rotations), 0, 1);
            Angle offset = Rotations.of(ExtraMath.lesser(modAngle-modCurrent, 1+modAngle-modCurrent, modAngle-modCurrent-1));

            Angle finalAngle = in.goal.plus(offset); 
            finalAngle = Radians.of((finalAngle.in(Radians)%
                (TURRET_ANGLE_LIM.in(Radians)*Math.signum(finalAngle.in(Radians)))
                ));
        
            io.setGoal(finalAngle);

            ready = in.atSetpoint;

            Logger.recordOutput("Subsystems/Turret/track/pathing/modAngle", modAngle);
            Logger.recordOutput("Subsystems/Turret/track/pathing/modCurrent", modCurrent);
            Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset", offset);
            Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset A", modAngle-modCurrent);
            Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset B", 1+modAngle-modCurrent);
            Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset c", modAngle-modCurrent-1);
            Logger.recordOutput("Subsystems/Turret/track/angle targeted", finalAngle);
            Logger.recordOutput("Subsystems/Turret/track/lead time", Seconds.of(timeOfFlight));
            Logger.recordOutput("Subsystems/Turret/track/target pos", targ.get());
            Logger.recordOutput("Subsystems/Turret/track/virtual target pos", virtualTarg);
            
        }, this).finallyDo(() -> {
            Logger.recordOutput("Subsystems/Turret/track/pathing/modAngle", Double.NaN, Rotations);
            Logger.recordOutput("Subsystems/Turret/track/pathing/modCurrent", Double.NaN, Rotations);
            Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset", Double.NaN, Rotations);
            Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset A", Double.NaN, Rotations);
            Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset B", Double.NaN, Rotations);
            Logger.recordOutput("Subsystems/Turret/track/pathing/angle offset c", Double.NaN, Rotations);
            Logger.recordOutput("Subsystems/Turret/track/angle targeted", Double.NaN, Rotations);
            Logger.recordOutput("Subsystems/Turret/track/lead time", Double.NaN, Seconds);
            Logger.recordOutput("Subsystems/Turret/track/target pos", (Translation2d)null);
            Logger.recordOutput("Subsystems/Turret/track/virtual target pos", (Translation2d)null);
        }).withName("track position");
    }

    public Command sysidQuasistatic(boolean reverse){
        return sysid.quasistatic(reverse ? SysIdRoutine.Direction.kReverse : SysIdRoutine.Direction.kForward)
        .until( () -> in.filteredAngle.abs(Radians) > TURRET_ANGLE_LIM.in(Radians))
        .raceWith(Commands.run(() -> {
            ready = false;
            runningSysid.set(true);
            runningSysid.setText("Turret sysid running: Quasistatic, " + (reverse ? "reverse" : "forward"));
        }))
        .finallyDo(() -> {
            runningSysid.set(false);
        })
        .withName("Quasistatic sysid: " + (reverse ? "reverse" : "forward"));
    }

    public Command sysidDynamic(boolean reverse){
        return sysid.dynamic(reverse ? SysIdRoutine.Direction.kReverse : SysIdRoutine.Direction.kForward)
        .until( () -> in.filteredAngle.abs(Radians) > TURRET_ANGLE_LIM.in(Radians))
        .raceWith(Commands.run(() -> {
            ready = false;
            runningSysid.set(true);
            runningSysid.setText("Turret sysid running: Dynamic, " + (reverse ? "reverse" : "forward"));
        }))
        .finallyDo(() -> {
            runningSysid.set(false);
        })
        .withName("Dynamic sysid: " + (reverse ? "reverse" : "forward"));
    }

    public boolean isReady(){
        return ready;
    }

    private Angle getAngleToPos(Translation2d target, Translation2d curr){
        return Radians.of(Math.atan2(target.getY()-curr.getY(), target.getX()-curr.getX()));
    }

    public Angle getAngle(){
        return in.filteredAngle;
    }

    public Translation2d getFieldPos(){
        return drive.getPose().getTranslation()//drive pos
            .plus(new Translation2d(//turret offest
                Math.cos(drive.getRotation().getRadians())*TURRET_OFFSET.getX(),
                Math.sin(drive.getRotation().getRadians())*TURRET_OFFSET.getX()));
    }

}
