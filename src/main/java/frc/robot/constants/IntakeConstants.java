package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import frc.utils.controlWrappers.PIDGains;

public class IntakeConstants {

    public static final int INTAKE_ROLLER_MOTOR_ID = 42;

    public static final int INTAKE_PIVOT_MOTOR_ID  = 52;
    public static final int INTAKE_PIVOT_ENCODER_ID = 34;

    public static final boolean INTAKE_ROLLER_INVERTED = false;
    public static final Current INTAKE_ROLLER_CURRENT_LIM = Amps.of(30);
    
    public static final boolean INTAKE_PIVOT_INVERTED = false;
    public static final Current INTAKE_PIVOT_CURRENT_LIM = Amps.of(30);

    public static final AngularVelocity INTAKE_RUN_VELOCITY = RPM.of(1000.0); // TODO: tune(10/kv should work)
    public static final AngularVelocity INTAKE_EJECT_VELOCITY = RPM.of(-500.0); // TODO: tune

    public static final PIDGains.SimpleFF INTAKE_ROLLER_FF_GAINS = new PIDGains.SimpleFF(0.0, 0.0, 0.0).makeTunable("Tuning/Intake/Roller/FF");
    
    //setpoints
    public static final Angle INTAKE_STOWED_ANGLE = Degrees.of(125);   // TODO: tune
    public static final Angle INTAKE_DEPLOYED_ANGLE = Degrees.of(0);   // TODO: tune

    //hard stops
    public static final Angle INTAKE_PIVOT_MIN_ANGLE = Degrees.of(-5);   // TODO: tune
    public static final Angle INTAKE_PIVOT_MAX_ANGLE = Degrees.of(130);   // TODO: tune

    public static final Angle INTAKE_PIVOT_TOLERANCE = Degrees.of(5);// TODO: tune

    public static final PIDGains.GravityFF INTAKE_PIVOT_ID_GAINS =new PIDGains.GravityFF(0.0, 0.1, 0.1, 0.1);

    public static final PIDGains.GravityFF INTAKE_PIVOT_FF_GAINS =new PIDGains.GravityFF(0.0, 0.1, 0.1, 0.1).makeTunable("Tuning/Intake/Pivot/FF");
    public static final PIDGains.ProfiledPID INTAKE_PIVOT_PID_GAINS = new PIDGains.ProfiledPID(0.0, 0.0, 0.0, 10.0, 15.0).makeTunable("Tuning/Intake/Pivot/PID");

    //ascope offset
    // public static final Translation3d INTAKE_OFFSET = new Translation3d(0.01373775,0,0.34845937);
    public static final Translation3d INTAKE_OFFSET = new Translation3d(0.292100,0,0.191);
}
