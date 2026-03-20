package frc.robot.constants;

import edu.wpi.first.units.measure.Distance;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.controlWrappers.PIDGains.ProfiledPID;
import frc.utils.controlWrappers.PIDGains.SimpleFF;

import static edu.wpi.first.units.Units.*;

public final class TurretConstants {
    
    public static final int TURRET_MOTOR_ID = 50;
    public static final int TURRET_ENCODER_1_ID = 31;
    public static final int TURRET_ENCODER_2_ID = 32;

    public static final double TURRET_MAIN_GEAR_TEETH = 200;
    public static final double TURRET_ENCODER_1_GEAR_TEETH = 35;
    public static final double TURRET_ENCODER_2_GEAR_TEETH = 34;
    public static final double TURRET_MOTOR_GEAR_TEETH = 20;

    public static final double SLOPE = (TURRET_ENCODER_2_GEAR_TEETH * TURRET_ENCODER_1_GEAR_TEETH)
            / ((TURRET_ENCODER_1_GEAR_TEETH - TURRET_ENCODER_2_GEAR_TEETH) * TURRET_MAIN_GEAR_TEETH);


    public static final Current TURRET_CURRENT_LIM = Amps.of(20);
    public static final boolean TURRET_MOTOR_INVERT = true;

    public static final Angle TURRET_ANGLE_FORWARD_LIM = Degrees.of(240);//soft limit before unwind(from center)
    public static final Angle TURRET_ANGLE_REVERSE_LIM = Degrees.of(-240);//soft limit before unwind(from center)

    public static final SimpleFF TURRET_ID_GAINS = new SimpleFF(0.3, 1.25, 0.00001);//gains from sysid for state space model

    public static final SimpleFF TURRET_FF_GAINS = new SimpleFF(0.4, 1.35, 0.0).makeTunable("Tuning/Turret/FF");
    public static final ProfiledPID TURRET_PID_GAINS = new ProfiledPID(7,1.0,0.3,1.5,10).makeTunable("Tuning/Turret/PID");
    public static final double TURRET_THETA_COMP_FACTOR = -0.08;//offset target angle while robot is spinning

    public static final Angle TURRET_SETPOINT_TOLERANCE = Degrees.of(5);

    public static final Distance HUB_RADIUS = Inches.of(45.7/2);
    public static final Distance PASS_RADIUS = Meters.of(0.75);

    public static final Translation2d RED_HUB = new Translation2d(11.915, 4.034);
    public static final Translation2d[] RED_PASS = new Translation2d[]{
        new Translation2d(14, 6.3),
        new Translation2d(14, 2.2)
    };
    
    public static final Translation2d BLUE_HUB = new Translation2d(4.625, 4.034);
    public static final Translation2d[] BLUE_PASS = new Translation2d[]{
        new Translation2d(2.5, 6.3),
        new Translation2d(2.5, 2.2)
    };

    public static final Translation3d TURRET_OFFSET = new Translation3d(-0.146050, 0.152400, 0.299237);
    public static final Translation3d HOOD_TO_TURRET_OFFSET = new Translation3d(0.090695,-0.00681,0.140578);

    public static final SysIdRoutine.Config TURRET_SYSID_CONFIG = new SysIdRoutine.Config(
        Volts.per(Second).of(1.0), 
        Volts.of(5.0), 
        Seconds.of(5),
        (state) -> Logger.recordOutput("Turret/SysIdTestState", state.toString())
    );
}
