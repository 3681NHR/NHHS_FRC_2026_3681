package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.utils.ExtraMath;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.RobotMode;

public class LaunchLUT {
    /**
     * hub LUT values
     * <p> must be sorted from smallest dist to largest
     */
    public static final ShotParams[] LUTHub = Constants.MODE == RobotMode.SIM ? new ShotParams[]{
        //  dist, hood, speed, time
        new ShotParams(Meters.of(1.516), Degrees.of(25), RPM.of(1150.0), Seconds.of(0.8)),
        new ShotParams(Meters.of(1.989), Degrees.of(27), RPM.of(1228.0), Seconds.of(0.9)),
        new ShotParams(Meters.of(2.3), Degrees.of(30), RPM.of(1228.0), Seconds.of(0.91)),
        new ShotParams(Meters.of(2.96), Degrees.of(30), RPM.of(1300.0), Seconds.of(0.92)),
        new ShotParams(Meters.of(3.77), Degrees.of(32), RPM.of(1450.0), Seconds.of(1.1)),
        new ShotParams(Meters.of(3.69), Degrees.of(32), RPM.of(1400.0), Seconds.of(1.11)),
        new ShotParams(Meters.of(5.33), Degrees.of(35), RPM.of(1550.0), Seconds.of(1.2)),
        new ShotParams(Meters.of(7.41), Degrees.of(45), RPM.of(1700.0), Seconds.of(1.4)),
        new ShotParams(Meters.of(9.86), Degrees.of(45), RPM.of(1900.0), Seconds.of(1.9)),
    } : new ShotParams[]{
        //  dist, hood, speed, time
        new ShotParams(Meters.of(1.6), Degrees.of(28), RPM.of(2500.0), Seconds.of(0.8)),
        new ShotParams(Meters.of(2.42), Degrees.of(32), RPM.of(2500.0), Seconds.of(1.05)),
        new ShotParams(Meters.of(2.9), Degrees.of(35), RPM.of(2500.0), Seconds.of(1.05)),
        new ShotParams(Meters.of(3.425), Degrees.of(35), RPM.of(2750.0), Seconds.of(1.1)),
        new ShotParams(Meters.of(4.07), Degrees.of(35), RPM.of(3000.0), Seconds.of(0.95)),
        new ShotParams(Meters.of(4.55), Degrees.of(35), RPM.of(3250.0), Seconds.of(0.95)),
        new ShotParams(Meters.of(5.04), Degrees.of(37), RPM.of(3500.0), Seconds.of(1.32)),
            new ShotParams(Meters.of(5.8), Degrees.of(45), RPM.of(4000), Seconds.of(1.15)),
            new ShotParams(Meters.of(6.36), Degrees.of(45), RPM.of(4000), Seconds.of(1.2)),
            new ShotParams(Meters.of(7.38), Degrees.of(45), RPM.of(5500), Seconds.of(1.4)),
    };
    /**
     * pass LUT values
     * <p> must be sorted from smallest dist to largest
     */
    public static final ShotParams[] LUTPass = Constants.MODE == RobotMode.SIM ? new ShotParams[]{
            //  dist, hood, speed, time
            new ShotParams(Meters.of(0), Degrees.of(35), RPM.of(1000.0), Seconds.of(0.0)),
    } : new ShotParams[]{
            //  dist, hood, speed, time
            new ShotParams(Meters.of(0), Degrees.of(35), RPM.of(2500.0), Seconds.of(0.0)),
    };

    /**
     * get data from LUT at given distance val
     * @param dist - value to lookup
     * @param lerp - weather to lerp between closest values or return nearest entry(will always return above dist)
     * @param LUT - array of params to use, sorted by distance
     * @return - array with data, [hood angle, launcher speed, TOF]
     * <p> edge cases:
     * <p> - when dist is greater than the farthest entry, farthest entry will be returned. 
     *          if lerp is true, returned value will be extrapolated from highest two entries
     * <p> - when dist is closer than minimum entry, the minumum entry will be returned
     */
    public static ShotParams get(Distance dist, boolean lerp, ShotParams[] LUT){
        ShotParams out = new ShotParams(Meters.of(0), Degrees.of(0), RPM.of(0.0), Seconds.of(0.0));
        if(LUT.length > 1 || !Double.isFinite(dist.baseUnitMagnitude())){

            //find upper bound of LUT
            int i=0;
            while(LUT[i].dist().lt(dist)){
                i++;
                //extrapolate if out of bounds
                if(i>=LUT.length){
                    Logger.recordOutput("lut/extrapolate", true);
                    if(!lerp && LUT.length >= 2){
                        return LUT[LUT.length-1];
                    } else {

                        double factor = 
                            dist.minus(LUT[LUT.length-2].dist())
                            .div(LUT[LUT.length-1].dist().minus(LUT[LUT.length-2].dist()))
                            .magnitude();

                        return new ShotParams(    //extrapolate based on last two values
                            dist,
                            Radians.of(ExtraMath.lerp(LUT[LUT.length-2].hoodAngle().in(Radians), LUT[LUT.length-1].hoodAngle().in(Radians), factor)),
                            RPM.of(    ExtraMath.lerp(LUT[LUT.length-2].speed().in(RPM)        , LUT[LUT.length-1].speed().in(RPM)        , factor)),
                            Seconds.of(ExtraMath.lerp(LUT[LUT.length-2].time().in(Seconds)     , LUT[LUT.length-1].time().in(Seconds)     , factor))
                        );
                    }
                }
           }
           Logger.recordOutput("lut/extrapolate", false);

            if(lerp && i>0){
                double factor = 
                    dist.minus(LUT[i-1].dist())
                    .div(LUT[i].dist().minus(LUT[i-1].dist()))
                    .magnitude();

                out = new ShotParams(
                    dist,
                    Radians.of(ExtraMath.lerp(LUT[i-1].hoodAngle().in(Radians), LUT[i].hoodAngle().in(Radians), factor)),
                    RPM.of(    ExtraMath.lerp(LUT[i-1].speed().in(RPM)        , LUT[i].speed().in(RPM)        , factor)),
                    Seconds.of(ExtraMath.lerp(LUT[i-1].time().in(Seconds)     , LUT[i].time().in(Seconds)     , factor))
                );
            } else {
                out = LUT[i];
            }

            Logger.recordOutput("lut/index", i);
        }
        Logger.recordOutput("lut/dist", dist);

        Logger.recordOutput("lut/params/dist", out.dist());
        Logger.recordOutput("lut/params/hood", out.hoodAngle());
        Logger.recordOutput("lut/params/speed", out.speed());
        Logger.recordOutput("lut/params/tof", out.time());
        
        return out;
    }
    
    /**
     * get slope of data from LUT at given distance val
     * @param dist - value to lookup
     * @param LUT - 2d array to use, needs to be n by 4 and sorted by distance
     * @return - params object with slope of LUT
     */
    public static ShotParams getSlope(Distance dist, ShotParams[] LUT){
        ShotParams out = new ShotParams(Meters.of(1), Degrees.of(1), RPM.of(1), Seconds.of(1));

        if(LUT.length > 1){
            int i=0;
            while(LUT[i].dist().lt(dist)){
                i++;
                if(i>=LUT.length){
                    Logger.recordOutput("lut/slope/extrapolate", true);

                    double run = LUT[LUT.length-2].dist().in(Meters)-LUT[LUT.length-1].dist().in(Meters);
                    return new ShotParams(    //extrapolate based on last two values
                        Meters.of(1),
                        Radians.of((LUT[LUT.length-2].hoodAngle().in(Radians) - LUT[LUT.length-1].hoodAngle().in(Radians))/run),
                        RPM.of(    (LUT[LUT.length-2].speed().in(RPM)         - LUT[LUT.length-1].speed().in(RPM))        /run),
                        Seconds.of((LUT[LUT.length-2].time().in(Seconds)      - LUT[LUT.length-1].time().in(Seconds))     /run)
                    );
                    
                }
            }
            Logger.recordOutput("lut/slope/extrapolate", false);

            i = Math.max(i, 1);

            double run = LUT[i-1].dist().in(Meters)-LUT[i].dist().in(Meters);
            out = new ShotParams(
                Meters.of(1),
                Radians.of((LUT[i-1].hoodAngle().in(Radians) - LUT[i].hoodAngle().in(Radians))/run),
                RPM.of(    (LUT[i-1].speed().in(RPM)         - LUT[i].speed().in(RPM))        /run),
                Seconds.of((LUT[i-1].time().in(Seconds)      - LUT[i].time().in(Seconds))     /run)
            );
            
            Logger.recordOutput("lut/slope/index", i);
        }
        Logger.recordOutput("lut/slope/dist", dist);

        Logger.recordOutput("lut/slope/params/dist", out.dist());
        Logger.recordOutput("lut/slope/params/hood", out.hoodAngle());
        Logger.recordOutput("lut/slope/params/speed", out.speed());
        Logger.recordOutput("lut/slope/params/tof", out.time());

        return out;
    }
    public record ShotParams(Distance dist, Angle hoodAngle, AngularVelocity speed, Time time){}
}