package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import frc.utils.ExtraMath;

public class LaunchLUT {
    public static final ShotParams[]LUTHub = {
    //  dist, hood, speed, time
        new ShotParams(Inches.of(3.5), Degrees.of(20), RPM.of(3107.0), Seconds.of(0.0)),
        new ShotParams(Inches.of(4), Degrees.of(20), RPM.of(2928.0), Seconds.of(0.0)),
        new ShotParams(Inches.of(6), Degrees.of(20), RPM.of(3178.0), Seconds.of(0.0)),
        new ShotParams(Inches.of(7.5), Degrees.of(20), RPM.of(3321.0), Seconds.of(0.0)),
        new ShotParams(Inches.of(8.5), Degrees.of(30), RPM.of(3190.0), Seconds.of(0.0)),
        new ShotParams(Inches.of(10), Degrees.of(30), RPM.of(3250.0), Seconds.of(0.0)),
        new ShotParams(Inches.of(11.5), Degrees.of(30), RPM.of(3500.0), Seconds.of(0.0)),
        new ShotParams(Inches.of(13), Degrees.of(30), RPM.of(3750.0), Seconds.of(0.0)),
        new ShotParams(Inches.of(15), Degrees.of(35), RPM.of(3857.0), Seconds.of(0.0)),
        new ShotParams(Inches.of(18), Degrees.of(45), RPM.of(3857.0), Seconds.of(0.0)),
    };

    /**
     * get data from LUT at given distance val
     * @param dist - value to lookup
     * @param lerp - weather to lerp between closest values or return nearest entry(will always return above dist)
     * @param LUT - 2d array to use, needs to be n by 4 and sorted by distance
     * @return - array with data, [hood angle, launcher speed, TOF]
     * <p> edge cases:
     * <p> - when dist is greater than the farthest entry, farthest entry will be returned. 
     *          if lerp is true, returned value will be extrapolated from highest two entries
     * <p> - when dist is closer than minimum entry, the minumum entry will be returned
     */
    public static ShotParams get(Distance dist, boolean lerp, ShotParams[] LUT){
        ShotParams out;

        int i=0;
        while(LUT[i].dist().gt(dist)){
            i++;
            if(i>=LUT.length){
                if(!lerp){
                    return LUT[LUT.length-1];
                } else {
                    double factor = (dist.in(Meters)-LUT[LUT.length-2].dist().in(Meters))/(LUT[LUT.length-1].dist().in(Meters)-LUT[LUT.length-2].dist().in(Meters));
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