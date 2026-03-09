package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.utils.controlWrappers.PIDGains;

public final class KickerConstants {
    public static final int KICKER_MOTOR_ID = 41;
    public static final int KICKER_SENSOR_ID = 33;

    public static final boolean KICKER_MOTOR_INVERT = false;
    public static final Current KICKER_MAX_CURRENT = Amps.of(20);

    public static final Distance KICKER_PRELOAD_STOP_DISTANCE = Centimeters.of(5);
    public static final Distance KICKER_PRELOAD_MAX_DISTANCE = Centimeters.of(22); // if the can range isnt detecting anything within this range, don't preload

    public static final LinearVelocity KICKER_PRELOAD_VELOCITY = Centimeters.per(Seconds).of(1);
    public static final LinearVelocity KICKER_FEED_VELOCITY = Centimeters.per(Seconds).of(1);

    public static final PIDGains.SimpleFF KICKER_ID_GAINS = new PIDGains.SimpleFF(0.0, 0.1, 0.001);

    public static final PIDGains.SimpleFF KICKER_FF_GAINS = new PIDGains.SimpleFF(0.0, 0.0, 0.0).makeTunable("Tuning/Kicker/FF");
    public static final PIDGains.PID KICKER_PID_GAINS = new PIDGains.PID(0.0, 0.0, 0.0).makeTunable("Tuning/Kicker/PID");

    public static final double POSITION_CONVERSION_FACTOR = (1.0/2.5)*2*Math.PI*Units.inchesToMeters(1);
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR*60;
}
