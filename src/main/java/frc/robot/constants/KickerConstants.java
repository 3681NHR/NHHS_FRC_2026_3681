package frc.robot.constants;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public final class KickerConstants {
    public static final int KICKER_MOTOR_ID = 41;
    public static final int KICKER_SENSOR_ID = 33;

    public static final boolean KICKER_MOTOR_INVERT = false;
    public static final Current KICKER_MAX_CURRENT = Amps.of(40);

    public static final Distance KICKER_PRELOAD_STOP_DISTANCE = Inches.of(3.5);
    public static final Distance KICKER_UNLOAD_MAX_DISTANCE = Inches.of(3.5); // if the can range isnt detecting anything within this range, don't preload

    public static final Voltage KICKER_UNLOAD_VOLTAGE = Volts.of(-6);
    public static final Voltage KICKER_UNLOAD_PARTIAL_VOLTAGE = Volts.of(-0);
    public static final Voltage KICKER_FEED_VOLTAGE = Volts.of(10);

    public static final double POSITION_CONVERSION_FACTOR = (1.0/2.5);
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR*60;
}
