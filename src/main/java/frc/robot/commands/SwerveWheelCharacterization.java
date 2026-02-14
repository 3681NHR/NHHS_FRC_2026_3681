package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.ArrayList;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DriveConstants;
import frc.robot.subsystems.swerve.Drive;
import frc.utils.ExtraMath;

/**
 * auto routine to calculate swerve wheel radius
 */
public class SwerveWheelCharacterization extends Command {
    Drive drive;

    ArrayList<AngularVelocity> gyroreadings = new ArrayList<AngularVelocity>();
    ArrayList<SwerveModuleState[]> wheelreadings = new ArrayList<SwerveModuleState[]>();

    Time startTimer = Seconds.of(1);

    public SwerveWheelCharacterization(Drive drive) {
        this.drive = drive;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.runAngleCharacterization(0);
    }

    @Override
    public void execute() {
        if(startTimer.in(Seconds) > 0){
            startTimer = startTimer.minus(Milliseconds.of(20));
            if(startTimer.in(Seconds) < 0.5){
                double out = 2;
                drive.runAngleCharacterization(out);
            }
        } else {
            double out = 2;
            drive.runAngleCharacterization(out);
            gyroreadings.add(drive.getAngulerVelocity());
            wheelreadings.add(drive.getModuleStates());
            double rad = (drive.getAngulerVelocity().in(RadiansPerSecond)*DriveConstants.RADIUS.in(Meters))
                /
                (ExtraMath.mean(
                    drive.getModuleStates()[0].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters),
                    drive.getModuleStates()[1].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters),
                    drive.getModuleStates()[2].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters),
                    drive.getModuleStates()[3].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters)
                    ));
            Logger.recordOutput("Wheel char/data/gyro radsPerSec", drive.getAngulerVelocity());
            Logger.recordOutput("Wheel char/data/wheel radPerSec", 2*Math.PI*ExtraMath.mean(
                    drive.getModuleStates()[0].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters),
                    drive.getModuleStates()[1].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters),
                    drive.getModuleStates()[2].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters),
                    drive.getModuleStates()[3].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters)
                    ));
            if(Double.isFinite(rad)){
                Logger.recordOutput("Wheel char/data/rad", Meters.of(rad).in(Inches));
            }

        }
        Logger.recordOutput("Wheel char/data/timer", startTimer);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
        assert(gyroreadings.size() == wheelreadings.size());
        ArrayList<Distance> rads = new ArrayList<Distance>();
        for(int i=0; i<gyroreadings.size(); i++){
            rads.add(
                Meters.of((gyroreadings.get(i).in(RadiansPerSecond)*DriveConstants.RADIUS.in(Meters))
                /
                (2*Math.PI*ExtraMath.mean(
                    wheelreadings.get(i)[0].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters),
                    wheelreadings.get(i)[1].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters),
                    wheelreadings.get(i)[2].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters),
                    wheelreadings.get(i)[3].speedMetersPerSecond/DriveConstants.module.WHEEL_RAD.in(Meters)
                    ))
                ));
        }
        double sum = 0;
        for(int i=0; i<rads.size(); i++){
            sum += rads.get(i).in(Meters);
        }
        sum = sum/rads.size();
        Logger.recordOutput("Wheel char/avg", Meters.of(sum));
        Logger.recordOutput("Wheel char/rads meters", rads.stream().mapToDouble(e -> e.in(Meters)).toArray());
    }

    @Override
    public boolean isFinished() {
        return DriverStation.isDisabled();
    }
}
