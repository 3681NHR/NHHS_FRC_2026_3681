package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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
    public static final double TURRET_ANGLE_LIM = 360;//soft limit before unwind(from center)

    public static final SimpleFF TURRET_ID_GAINS = new SimpleFF(0.2,1,0.01);//gains from sysid for state space model

    public static final SimpleFF TURRET_FF_GAINS = TURRET_ID_GAINS;
    public static final ProfiledPID TURRET_PID_GAINS = new ProfiledPID(0.5,0,0,10,10);

    public static final State TURRET_SETPOINT_TOLERANCE = new State(Units.degreesToRadians(1), 1);

    public static final Translation2d RED_HUB = new Translation2d();
    public static final Translation2d RED_PASS_L = new Translation2d();
    public static final Translation2d RED_PASS_R = new Translation2d();
    
    public static final Translation2d BLUE_HUB = new Translation2d();
    public static final Translation2d BLUE_PASS_L = new Translation2d();
    public static final Translation2d BLUE_PASS_R = new Translation2d();

}
