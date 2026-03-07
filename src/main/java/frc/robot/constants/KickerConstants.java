package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.utils.controlWrappers.PIDGains;

public final class KickerConstants {
    public static final int KICKER_MOTOR_ID = -1;
    public static final int KICKER_CAN_RANGE_ID = -1;

    public static final boolean MOTOR_INVERT = false;

    public static final Distance PRELOAD_DISTANCE_THRESHOLD = Centimeters.of(5);
    public static final Distance PRELOAD_MAX_DISTANCE = Centimeters.of(22); // if the can range isnt detecting anything within this range, don't preload - "evans idea"
    public static final LinearVelocity PRELOAD_VELOCITY = Centimeters.per(Second).of(0);
    public static final LinearVelocity SHOOT_VELOCITY = Centimeters.per(Second).of(0);

    public static final PIDGains.PID KICKER_PID_GAINS = new PIDGains.PID(0.0, 0.0, 0.0);
    public static final PIDGains.SimpleFF KICKER_FF_GAINS = new PIDGains.SimpleFF(0.0, 0.0, 0.0);

    public static final Current KICKER_MAX_CURRENT = Amps.of(20);

    public static final double POSITION_CONVERSION_FACTOR = (1.0/2.5)*2*Math.PI*Units.inchesToMeters(1);
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR;
}
