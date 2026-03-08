package frc.robot.constants;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import frc.utils.controlWrappers.PIDGains;

public class ClimbConstants {
    public static final int MOTOR_ID = 5;
    public static final boolean INVERTED = false;
    public static final Distance EXTEND_POSITION = Meters.one();
    public static final PIDGains.ProfiledPID CLIMB_PID_GAINS = new PIDGains.ProfiledPID(1, 1, 1, 100, 100).makeTunable("Tuning/Climber/PID");
    public static final PIDGains.GravityFF FF = new PIDGains.GravityFF(0, 0, 0, 0).makeTunable("Tuning/Climber/FF");
    public static final PIDGains.GravityFF CLIMB_ID_GAINS = new PIDGains.GravityFF(0.1, 0.1, 0.1, 0.01);
    private static final double GEAR_RATIO = 80.0;
    private static final Distance SPOOL_RADIUS = Meters.one(); // get actual circumference later
    public static final double POSITION_CONVERSION_FACTOR = (SPOOL_RADIUS.in(Meters)*2*Math.PI)/GEAR_RATIO;
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR;
    public static final Distance MIN_POSITION = Meters.zero();
    public static final Distance SETPOINT_TOLERANCE = Meters.of(0.05); // in meters, placeholder value
}
