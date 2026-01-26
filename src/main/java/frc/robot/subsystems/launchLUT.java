package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import frc.utils.ExtraMath;

public class launchLUT {
    public static final double[][] LUTGround = {
    //  dist, hood, speed, time
        {0.0, 0.0, 1.0, 0.0},
        {2.0, 1.0, 2.0, 2.0}
    };
    public static final double[][] LUTHub = {
    //  dist, hood, speed, time
        {Units.feetToMeters(18), Units.degreesToRadians(45), 3857.0, 1.0},
        {Units.feetToMeters(15), Units.degreesToRadians(35), 3857.0, 1.35},
        {Units.feetToMeters(13), Units.degreesToRadians(30), 3750.0, 1.26},
        {Units.feetToMeters(11.5), Units.degreesToRadians(30), 3500.0, 1.23},
        {Units.feetToMeters(10), Units.degreesToRadians(30), 3250.0, 1.0},
        {Units.feetToMeters(8.5), Units.degreesToRadians(30), 3190.0, 1.0},
        {Units.feetToMeters(7.5), Units.degreesToRadians(20), 3321.0, 1.3},
        {Units.feetToMeters(6), Units.degreesToRadians(20), 3178.0, 1.03},
        {Units.feetToMeters(4), Units.degreesToRadians(20), 2928.0, 1.1},
        {Units.feetToMeters(4), Units.degreesToRadians(20), 2928.0, 1.1},
        {Units.feetToMeters(3.5), Units.degreesToRadians(20), 3107.0, 1.2},
    };

    public static double[] get(double dist, boolean lerp, double[][] LUT){
        double[] out = new double[3];

        int i=0;
        while(LUT[i][0]<dist){
            i++;
            if(i>=LUT.length){
                throw new IndexOutOfBoundsException("distance value out of LUT bounds");
            }
        }
        if(lerp && i>0){
            out[0] = ExtraMath.lerp(LUT[i-1][1], LUT[i][1], (dist-LUT[i-1][0])/(LUT[i][0]-LUT[i-1][0]));
            out[1] = ExtraMath.lerp(LUT[i-1][2], LUT[i][2], (dist-LUT[i-1][0])/(LUT[i][0]-LUT[i-1][0]));
            out[2] = ExtraMath.lerp(LUT[i-1][3], LUT[i][3], (dist-LUT[i-1][0])/(LUT[i][0]-LUT[i-1][0]));
        } else {
            out[0] = LUT[i][1];
            out[1] = LUT[i][2];
            out[2] = LUT[i][3];
        }
        return out;
    }
}
