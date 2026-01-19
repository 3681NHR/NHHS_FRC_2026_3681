package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc.utils.controlWrappers.PIDGains.ProfiledPID;
import frc.utils.controlWrappers.PIDGains.SimpleFF;

public final class TurretConstants {
    
    public static final int TURRET_MOTOR_ID = -1;
    public static final int TURRET_ENCODER_1_ID = -1;
    public static final int TURRET_ENCODER_2_ID = -1;

    public static final int TURRET_MAIN_GEAR_TEETH = 200;
    public static final int TURRET_ENCODER_1_GEAR_TEETH = 35;
    public static final int TURRET_ENCODER_2_GEAR_TEETH = 34;

    public static final double TURRET_ANGLE_OFFSET = 0;
    public static final double TURRET_ANGLE_LIM = Units.degreesToRadians(360);//soft limit before unwind(from center)

    public static final SimpleFF TURRET_ID_GAINS = new SimpleFF(0.2,0.25,0.01);//gains from sysid for state space model

    public static final SimpleFF TURRET_FF_GAINS = TURRET_ID_GAINS;
    public static final ProfiledPID TURRET_PID_GAINS = new ProfiledPID(3.0,0,1.0,20,100);
    public static final double TURRET_THETA_COMP_FACTOR = -0.08;

    public static final double TURRET_SETPOINT_TOLERANCE = Units.degreesToRadians(5);

    public static final Translation2d RED_HUB = new Translation2d(11.915, 4.034);
    public static final Translation2d[] RED_PASS = new Translation2d[]{
        new Translation2d(13, 6.5),
        new Translation2d(13, 2)
    };
    
    public static final Translation2d BLUE_HUB = new Translation2d(4.625, 4.034);
    public static final Translation2d[] BLUE_PASS = new Translation2d[]{
        new Translation2d(3.5, 6.5),
        new Translation2d(3.5, 2)
    };

    public static final double TURRET_LOCK_POS = 0.0;

    public static final Translation3d TURRET_OFFSET = new Translation3d(-.158750,0,0.298450);
    public static final Translation3d HOOD_TO_TURRET_OFFSET = new Translation3d(0.085914,0,0.141886);

}
