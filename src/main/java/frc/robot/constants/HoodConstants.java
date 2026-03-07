package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.utils.controlWrappers.PIDGains;

public final class HoodConstants {
    public static final int HOOD_MOTOR_ID = 51;

    public static final boolean HOOD_HOME_ON_START = false;

    public static final Angle HOOD_SETPOINT_TOLERANCE = Degrees.of(1);
    
    public static final Angle HOOD_MIN_ANGLE = Degrees.of(25);
    public static final Angle HOOD_MAX_ANGLE = Degrees.of(52);

    public static final PIDGains.SimpleFF HOOD_ID_GAINS = new PIDGains.SimpleFF(0.3,35,0.0000001);//from sysid

    public static final PIDGains.ProfiledPID HOOD_PID_GAINS = new PIDGains.ProfiledPID(35, 15, 5, 0.2, 3).makeTunable("Tuning/Hood/PID");
    public static final PIDGains.SimpleFF HOOD_FF_GAINS = new PIDGains.SimpleFF(0.3,35,0.0).makeTunable("Tuning/Hood/FF");

    public static final Voltage HOOD_HOME_VOLTAGE = Volts.of(-2);
    public static final Time HOOD_HOME_STOP_TIME = Seconds.of(0.5);
    public static final AngularVelocity HOOD_HOME_STOP_THRESH = RadiansPerSecond.of(0.01);

    public static final double HOOD_GEAR_RATIO = (1.0/25.0)*(10.0/176.0)*0.750671851307;

    public static final Current HOOD_CURRENT_LIM = Amps.of(20);
}
