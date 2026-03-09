package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.utils.controlWrappers.PIDGains;

public class ClimberConstants {
    public static final int CLIMBER_MOTOR_ID = 5;

    public static final boolean CLIMBER_INVERTED = false;
    public static final Current CLIMBER_CURRENT_LIM = Amps.of(30);

    public static final Distance CLIMBER_MAX_POSITION = Meters.of(1);
    public static final Distance CLIMBER_MIN_POSITION = Meters.of(0);

    public static final Distance CLIMBER_SETPOINT_TOLERANCE = Inches.of(1);

    public static final PIDGains.GravityFF CLIMBER_ID_GAINS = new PIDGains.GravityFF(0.1, 0, 0.1, 0.0001);

    public static final PIDGains.GravityFF CLIMBER_FF_GAINS = new PIDGains.GravityFF(0.1, 0, 0.1, 0).makeTunable("Tuning/Climber/FF");
    public static final PIDGains.ProfiledPID CLIMBER_PID_GAINS = new PIDGains.ProfiledPID(1, 1, 1, 100, 100).makeTunable("Tuning/Climber/PID");

    private static final Distance CLIMBER_SPOOL_RADIUS = Inches.of(1); //TODO: get actual radius later
    private static final double CLIMBER_GEAR_RATIO = 80.0;
    public static final double CLIMBER_POSITION_CONVERSION_FACTOR = (CLIMBER_SPOOL_RADIUS.in(Meters)*2*Math.PI)/CLIMBER_GEAR_RATIO;
    public static final double CLIMBER_VELOCITY_CONVERSION_FACTOR = CLIMBER_POSITION_CONVERSION_FACTOR;

    
    public static final boolean CLIMBER_HOME_ON_START = false; 
    public static final Voltage CLIMBER_HOME_VOLTAGE = Volts.of(-2);
    public static final Time CLIMBER_HOME_STOP_TIME = Seconds.of(0.5);
    public static final LinearVelocity CLIMBER_HOME_STOP_THRESH = MetersPerSecond.of(0.01);
}
