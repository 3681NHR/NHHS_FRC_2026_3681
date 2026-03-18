package frc.utils;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radians;
import static frc.robot.constants.TurretConstants.*;

import java.lang.Math;
import java.util.ArrayList;

/*
 * extra math utils
 */
public final class ExtraMath {

    public static double calculateTurretAngleFromCANCoderDegrees(double e1, double e2) {
        double difference = e2 - e1;
        if (difference > 180) {
            difference -= 360;
        }
        if (difference < -180) {
            difference += 360;
        }
        difference *= SLOPE;

        double e1Rotations = (difference * TURRET_MAIN_GEAR_TEETH / TURRET_ENCODER_1_GEAR_TEETH) / 360.0;
        double e1RotationsFloored = Math.floor(e1Rotations);
        double turretAngle = (e1RotationsFloored * 360.0 + e1) * (TURRET_ENCODER_1_GEAR_TEETH / TURRET_MAIN_GEAR_TEETH);

        Logger.recordOutput("fahhh/angle", turretAngle);
        if (turretAngle - difference < -40) {
            turretAngle += TURRET_ENCODER_1_GEAR_TEETH / TURRET_MAIN_GEAR_TEETH * 360.0;
        } else if (turretAngle - difference > 40) {
            turretAngle -= TURRET_ENCODER_1_GEAR_TEETH / TURRET_MAIN_GEAR_TEETH * 360.0;
        }
        Logger.recordOutput("fahhh/dif", difference);
        return turretAngle;
    }

    public static <T extends Measure<? extends Unit>> T clamp(T val, T min, T max){
        if(val.baseUnitMagnitude() >= max.baseUnitMagnitude()){
            return max;
        }
        if(val.baseUnitMagnitude() <= min.baseUnitMagnitude()){
            return min;
        }
        return val;
    }

    public static Angle getAngleToPos(Translation2d target, Translation2d curr){
        return Radians.of(Math.atan2(target.getY()-curr.getY(), target.getX()-curr.getX()));
    }

    public static double mean(double... in){
        double sum = 0;
        for(double i : in){
            sum += i;
        }
        return sum/in.length;
    }
    /**
     * @param in values to compare
     * @return value with smallest absolute value
     */
    public static double lesser(double... in){
        double lesser = Double.POSITIVE_INFINITY;
        double lesserAbs = Double.POSITIVE_INFINITY;
        for (double v : in) {
            if (Math.abs(v) < lesserAbs) {
                lesser = v;
                lesserAbs = Math.abs(v);
            }
        }
        return lesser;
    }
    
    /**
     * @param in values to compare
     * @return value with largest absolute value
     */
    public static double greater(double... in){
        double greater = Double.NEGATIVE_INFINITY;
        double greaterAbs = 0.0;
        for (double v : in) {
            if (Math.abs(v) > greaterAbs) {
                greater = v;
                greaterAbs = Math.abs(v);
            }
        }
        return greater;
    }
    
    public static double min(double... in){
        double lesser = Double.POSITIVE_INFINITY;
        double lesserAbs = Double.POSITIVE_INFINITY;
        for (double v : in) {
            if (v < lesserAbs) {
                lesser = v;
                lesserAbs = v;
            }
        }
        return lesser;
    }
    
    public static double max(double... in){
        double greater = Double.NEGATIVE_INFINITY;
        double greaterAbs = 0.0;
        for (double v : in) {
            if (v > greaterAbs) {
                greater = v;
                greaterAbs = v;
            }
        }
        return greater;
    }

    public static boolean isNearState(State expected, State actual, State tolerance){
        return MathUtil.isNear(expected.position, actual.position, tolerance.position) && MathUtil.isNear(expected.velocity, actual.velocity, tolerance.velocity);
    }
    /**
     * round to n decimal places
     */
    public static double roundToPoint(double val, int point) {
        return (int)(val*Math.pow(10, point)) / Math.pow(10, point);
    }

    /**
     * get rotation2D angle from 0,0 to u,v
     */
    public static Rotation2d getAngle(double u, double v) {
        return new Rotation2d(Math.atan2(v, u));
    }

    /**
     * get distance between 0,0 and x,y
     */
    public static double getMagnitude(double x, double y) {
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    /**
     * get tilt of robot
     * 
     * @param angle 3d gyro angle of bot
     * @return double[2] index 0 is direction(-pi to pi), index 1 is tilt angle
     */
    public static double[] getTip(Rotation3d angle) {
        double[] out = new double[2];

        out[0] = Math.atan2(-angle.getX(), -angle.getY());
        out[1] = Math.hypot(angle.getX(), angle.getY());

        return out;
    }

    /**
     * process input value
     * curve function graphed
     * {@link <a href="https://www.desmos.com/calculator/fjuc4iqjqt">here</a>}
     *
     * @param val        - number to process
     * @param multiplier - multiplier for input, mainly used for inverting
     * @param square     - polynomial curve value, roughly y=x^s
     *                   {@link <a href="https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#squaring-inputs">...</a>}
     * @param deadZone   - deadzone for input
     *                   {@link <a href="https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html#input-deadband">...</a>}
     * @return processed value
     */
    public static double processInput(Double val, Double multiplier, Double square, Double deadZone) {
        double out = val;

        if (square != null) {
            if (deadZone != null && deadZone > 0) {
                out = Math.signum(val)
                        * Math.pow((1 / (-deadZone + 1)) * Math.abs(val) - (deadZone / (-deadZone + 1)), square);
            } else {
                out = Math.signum(val) * Math.pow(Math.abs(val), square);
            }
        }
        if (deadZone != null) {
            out = MathUtil.applyDeadband(out, deadZone);
        }
        if (multiplier != null) {
            out *= multiplier;
        }

        return out;
    }

    public static double remap(double val, double inMin, double inMax, double outMin, double outMax) {
        return ((val - inMin) / (inMax - inMin) * (outMax - outMin)) + outMin;
    }

    public static double lerp(double start, double end, double val) {
        return (end - start) * val + start;
    }
    /**
     * lerp rgb color
     */
    public static Color colLerp(Color start, Color end, double val) {
        double r = lerp(start.red, end.red, val);
        double g = lerp(start.green, end.green, val);
        double b = lerp(start.blue, end.blue, val);
        return new Color(r, g, b);
    }

    public static Color rgbToHsv(Color in) {
        float[] hsv = new float[3];
        java.awt.Color.RGBtoHSB((int)(in.red*255), (int)(in.green*255), (int)(in.blue*255), hsv);
        return new Color(hsv[0], hsv[1], hsv[2]);
    }

    /**
     * get nearest pose from array to current
     */
    public static Pose2d getNearestPose(Pose2d[] poses, Pose2d current) {
        double min = Double.MAX_VALUE;
        Pose2d out = poses[0];
        for (Pose2d pose : poses) {
            double dist = current.getTranslation().getDistance(pose.getTranslation());
            dist += Math.abs(current.getRotation().minus(pose.getRotation()).getRadians());
            dist = Math.abs(dist / 2.0);
            if (dist < min) {
                min = dist;
                out = pose;
            }
        }
        return out;
    }
    /**
     * get translation distance between two pose2Ds
     */
    public static double getDistance(Pose2d a, Pose2d b) {
        return a.getTranslation().getDistance(b.getTranslation());
    }

    /**
     * check if two poses are within tolerance in translation and rotation
     */
    public static boolean poseWithinTolerance(Pose2d a, Pose2d b, double toleranceLinear, double toleranceAngular) {

        double dist = getDistance(a, b);
        double angleDiff = Math.abs(a.getRotation().minus(b.getRotation()).getRadians());

        return dist < toleranceLinear && angleDiff < toleranceAngular;
    }

    public static Color normalizeCol(Color in) {
        double brt = in.red / 255.0 + in.green / 255.0 + in.blue / 255.0;
        return new Color(in.red / brt, in.green / brt, in.blue / brt);
    }

    /**
     * Derivitive class
     */
    public static class Derivitive {

        private double oldValue;
        private boolean init;

        /**
         * constructs a new Derivitive object, first update will be based off initial
         * mesurement
         * 
         * @param initMesure initial mesurement
         */
        public Derivitive(double initMesure) {
            oldValue = initMesure;
            init = true;
        }

        /**
         * constructs a new Derrivitive object, first update will return 0
         */
        public Derivitive() {
            init = false;
        }

        /**
         * calculate the rate of change of a mesurement
         * 
         * @param mesurement current mesurement
         * @param dt         time since last update
         * @return rate of change
         */
        public double calculate(double mesurement, double dt) {
            if (!init) {
                oldValue = mesurement;
                init = true;
            }
            double out = (mesurement - oldValue) / dt;
            oldValue = mesurement;
            return out;
        }

        public void reset(double initMesure) {
            oldValue = initMesure;
        }
    }

    public static class MovingAverageFilter {
        private final ArrayList<Double> window = new ArrayList<>();
        private final int taps;

        public MovingAverageFilter(int taps) {
            this.taps = taps;
        }

        public double calculate(double in) {
            window.add(in);
            if (window.size() > taps) {
                window.remove(0);
            }
            double t = 0;
            for (double x : window) {
                t += x;
            }

            t = t / window.size();

            return t;
        }

        public void reset() {
            window.clear();
        }
    }
}
