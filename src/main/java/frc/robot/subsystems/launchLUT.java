package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
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

public class LaunchLUT {
    /**
     * hub LUT values from sim
     */
    public static final ShotParams[]LUTHub = {
    //  dist, hood, speed, time
        new ShotParams(Meters.of(1.516), Degrees.of(25), RPM.of(1150.0), Seconds.of(0.8)),
        new ShotParams(Meters.of(1.989), Degrees.of(27), RPM.of(1228.0), Seconds.of(0.9)),
        new ShotParams(Meters.of(2.3), Degrees.of(30), RPM.of(1228.0), Seconds.of(0.9)),
        new ShotParams(Meters.of(2.96), Degrees.of(30), RPM.of(1300.0), Seconds.of(0.9)),
        new ShotParams(Meters.of(3.77), Degrees.of(32), RPM.of(1450.0), Seconds.of(1.1)),
        new ShotParams(Meters.of(5.33), Degrees.of(35), RPM.of(1550.0), Seconds.of(1.1)),
        new ShotParams(Meters.of(7.41), Degrees.of(45), RPM.of(1700.0), Seconds.of(1.4)),
        new ShotParams(Meters.of(9.86), Degrees.of(45), RPM.of(1900.0), Seconds.of(1.9)),
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
        ShotParams out;

        int i=0;
        while(LUT[i].dist().lt(dist)){
            i++;
            if(i>=LUT.length){
                if(!lerp){
                    return LUT[LUT.length-1];
                } else {
                    double factor = (dist.in(Meters)-LUT[LUT.length-2].dist().in(Meters))/(LUT[LUT.length-1].dist().in(Meters)-LUT[LUT.length-2].dist().in(Meters));
                    Logger.recordOutput("lut/extrapolate", true);
                    return new ShotParams(    //extrapolate based on last two values
                    dist,
                    Radians.of(ExtraMath.lerp(LUT[LUT.length-2].hoodAngle().in(Radians), LUT[LUT.length-1].hoodAngle().in(Radians), factor)),
                    RPM.of(ExtraMath.lerp(LUT[LUT.length-2].speed().in(RPM), LUT[LUT.length-1].speed().in(RPM), factor)),
                    Seconds.of(ExtraMath.lerp(LUT[LUT.length-2].time().in(Seconds), LUT[LUT.length-1].time().in(Seconds), factor))
                    );
                }
            }
        }
            
        if(lerp && i>0){
            double factor = (dist.in(Meters)-LUT[i-1].dist().in(Meters))/(LUT[i].dist().in(Meters)-LUT[i-1].dist().in(Meters));
            out = new ShotParams(
                dist,
                Radians.of(ExtraMath.lerp(LUT[i-1].hoodAngle().in(Radians), LUT[i].hoodAngle().in(Radians), factor)),
                RPM.of(ExtraMath.lerp(LUT[i-1].speed().in(RPM), LUT[i].speed().in(RPM), factor)),
                Seconds.of(ExtraMath.lerp(LUT[i-1].time().in(Seconds), LUT[i].time().in(Seconds), factor))
            );
        } else {
            out = LUT[i];
        }
        Logger.recordOutput("lut/extrapolate", false);

        Logger.recordOutput("lut/dist", dist);
        Logger.recordOutput("lut/index", i);
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
     * @return - array with data, [hood angle, launcher speed, TOF]
     */
    public static ShotParams getSlope(Distance dist, ShotParams[] LUT){
        ShotParams out;

        int i=0;
        while(LUT[i].dist().gt(dist)){
            i++;
            if(i>=LUT.length){
                double run = LUT[LUT.length-2].dist().in(Meters)-LUT[LUT.length-1].dist().in(Meters);
                return new ShotParams(    //extrapolate based on last two values
                    Meters.of(1),
                    Radians.of((LUT[LUT.length-2].hoodAngle().in(Radians) - LUT[LUT.length-1].hoodAngle().in(Radians))/run),
                    RPM.of((LUT[LUT.length-2].speed().in(RPM) - LUT[LUT.length-1].speed().in(RPM))/run),
                    Seconds.of((LUT[LUT.length-2].time().in(Seconds) - LUT[LUT.length-1].time().in(Seconds))/run)
                );
                
            }
        }
        if(i>0){
            double run = LUT[i-1].dist().in(Meters)-LUT[i].dist().in(Meters);
            out = new ShotParams(
                Meters.of(1),
                Radians.of((LUT[i-1].hoodAngle().in(Radians) - LUT[i].hoodAngle().in(Radians))/run),
                RPM.of((LUT[i-1].speed().in(RPM) - LUT[i].speed().in(RPM))/run),
                Seconds.of((LUT[i-1].time().in(Seconds) - LUT[i].time().in(Seconds))/run)
            );
        } else {
            out = LUT[i];
        }
        return out;
    }
    public record ShotParams(Distance dist, Angle hoodAngle, AngularVelocity speed, Time time){}
}