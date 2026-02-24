package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.launchLUT.ShotParams;
import frc.robot.subsystems.swerve.Drive;
import frc.utils.ExtraMath;

public class SOTMSolver extends SubsystemBase{
    private static SOTMSolver instance;
    private ShotParams params;
    private Translation2d target = new Translation2d();
    private Drive drive;

    private SOTMSolver(){
        calculate();
    };

    public synchronized static SOTMSolver getInstance(){
        if(instance == null){
            instance = new SOTMSolver();
        }
        return instance;
    }

    @Override 
    public void periodic(){
        calculate();
    }

    public void setTarget(Translation2d targ){
        this.target = targ;
    }

    public void setDrive(Drive drive){
        this.drive = drive;
    }
    
    public void calculate(){
        Translation2d curr = drive.getPose().getTranslation();
        Translation2d vel = new Translation2d(
            drive.getChassisSpeeds().vxMetersPerSecond,
            drive.getChassisSpeeds().vyMetersPerSecond
        );
        Translation2d shotVel = new Translation2d(
            Math.cos(ExtraMath.getAngleToPos(target, curr).in(Radians)),
            Math.sin(ExtraMath.getAngleToPos(target, curr).in(Radians))
        ).times(curr.getDistance(target) * launchLUT.get(Meters.of(curr.getDistance(target)), true, launchLUT.LUTHub).time().in(Seconds));
        Translation2d targetVel = shotVel.minus(vel);
        double targetV = targetVel.getDistance(new Translation2d());
        double currentV = params.dist().in(Meters);
        double dist = params.dist().in(Meters);

        for(int i=0; i<10 && Math.abs(currentV - targetV) > 0.005; i++){
            double dv = (dist*launchLUT.getSlope(Meters.of(dist), launchLUT.LUTHub).time().in(Seconds))/Math.pow(launchLUT.get(Meters.of(dist), true, launchLUT.LUTHub).time().in(Seconds), 2);
            dist -= (currentV-targetV)/dv;
            currentV = dist/launchLUT.get(Meters.of(dist), true, launchLUT.LUTHub).time().in(Seconds);
        }
        params = launchLUT.get(Meters.of(dist), true, launchLUT.LUTHub);

    }

    public ShotParams getParams(boolean refresh){
        if(refresh){
            calculate();
        }
        return params;
    }
}
