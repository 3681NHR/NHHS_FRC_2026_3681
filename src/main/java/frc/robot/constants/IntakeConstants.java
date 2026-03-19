package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.utils.controlWrappers.PIDGains;

public class IntakeConstants {

    public static final int INTAKE_ROLLER_MOTOR_ID = 42;

    public static final int INTAKE_PIVOT_MOTOR_ID  = 52;
    public static final int INTAKE_PIVOT_ENCODER_ID = 34;

    public static final boolean INTAKE_ROLLER_INVERTED = false;
    public static final Current INTAKE_ROLLER_CURRENT_LIM = Amps.of(30);
    
    public static final boolean INTAKE_PIVOT_INVERTED = false;
    public static final Current INTAKE_PIVOT_CURRENT_LIM = Amps.of(40);

    public static final Voltage INTAKE_RUN_VOLTAGE = Volts.of(11); // TODO: tune
    public static final Voltage INTAKE_EJECT_VOLTAGE = Volts.of(-10); // TODO: tune

    //setpoints
    public static final Angle INTAKE_STOWED_ANGLE = Degrees.of(125);   // TODO: tune
    public static final Angle INTAKE_DEPLOYED_ANGLE = Degrees.of(0);   // TODO: tune

    //hard stops for sim
    public static final Angle INTAKE_PIVOT_MIN_ANGLE = Degrees.of(-5); 
    public static final Angle INTAKE_PIVOT_MAX_ANGLE = Degrees.of(130); 

    public static final Angle INTAKE_PIVOT_TOLERANCE = Degrees.of(5);// TODO: tune

    public static final PIDGains.GravityFF INTAKE_PIVOT_ID_GAINS =new PIDGains.GravityFF(0.0, 0.3, 0.7, 0.001);

    public static final PIDGains.GravityFF INTAKE_PIVOT_FF_GAINS =new PIDGains.GravityFF(0.0, 0.3, 0.7, 0.0).makeTunable("Tuning/Intake/Pivot/FF");
    public static final PIDGains.ProfiledPID INTAKE_PIVOT_PID_GAINS = new PIDGains.ProfiledPID(0.3, 0.00067, 0.2, 7.0, 15.0).makeTunable("Tuning/Intake/Pivot/PID");

    //ascope offset
    public static final Translation3d INTAKE_OFFSET = new Translation3d(0.292100,0,0.191);
}
