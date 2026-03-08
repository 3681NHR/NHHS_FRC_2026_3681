package frc.robot.constants;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.utils.controlWrappers.PIDGains;

public class IntakeConstants {

    //  CAN IDs 
    public static final int INTAKE_MOTOR_ID = 42; // TODO: set real CAN ID
    public static final int PIVOT_MOTOR_ID  = 52; // TODO: set real CAN ID
    public static final int INTAKE_ENCODER_ID = 34; // TODO: set real CAN ID
    //  Roller 
    public static final boolean ROLLER_INVERTED           = false;
    public static final int     ROLLER_SMART_CURRENT_LIMIT = 40; // amps

    /** Velocity setpoint tolerance. */
    public static final AngularVelocity ROLLER_TOLERANCE = RPM.of(50.0);

    /** Intake velocity setpoint. */
    public static final AngularVelocity INTAKE_VELOCITY = RPM.of(1000.0); // TODO: tune

    /** Eject velocity setpoint (negative = reverse). */
    public static final AngularVelocity EJECT_VELOCITY = RPM.of(-500.0); // TODO: tune

    public static final PIDGains.SimpleFF ROLLER_FF_GAINS =
            new PIDGains.SimpleFF(0.0, 0.0, 0.0)
                    .makeTunable("Tuning/Intake/Roller/FF");

    //  Pivot 
    public static final boolean PIVOT_INVERTED            = false;
    public static final int     PIVOT_SMART_CURRENT_LIMIT = 30; // amps

    /** Gear ratio between the motor and the pivot joint. */
    private static final double PIVOT_GEAR_RATIO = 1.0; // TODO: set real gear ratio

    /** Converts encoder rotations -> radians at the pivot joint. */
    public static final double PIVOT_POSITION_CONVERSION_FACTOR = (2.0 * Math.PI) / PIVOT_GEAR_RATIO;

    /** Converts encoder RPM -> rad/s at the pivot joint. */
    public static final double PIVOT_VELOCITY_CONVERSION_FACTOR = PIVOT_POSITION_CONVERSION_FACTOR / 60.0;

    /** Goal angle tolerance. */
    public static final Angle PIVOT_TOLERANCE = Radians.of(0.05); // ~3 degrees

    /** Stowed (retracted) pivot angle. */
    public static final Angle STOWED_ANGLE   = Radians.of(0.0);   // TODO: tune

    /** Deployed (floor-facing) pivot angle. */
    public static final Angle DEPLOYED_ANGLE = Radians.of(1.5);   // TODO: tune (~86 degrees)

    public static final PIDGains.ProfiledPID PIVOT_PID_GAINS =
            new PIDGains.ProfiledPID(0.0, 0.0, 0.0, 5.0, 10.0)
                    .makeTunable("Tuning/Intake/Pivot/PID");

    public static final PIDGains.GravityFF PIVOT_FF_GAINS =
            new PIDGains.GravityFF(0.0, 0.0, 0.0, 0.0)
                    .makeTunable("Tuning/Intake/Pivot/FF");
}
