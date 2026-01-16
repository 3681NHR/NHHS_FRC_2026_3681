package frc.robot.constants;

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

    public static final SimpleFF TURRET_FF = new SimpleFF(0,0,0);
    public static final ProfiledPID TURRET_PID_GAINS = new ProfiledPID(0,0,0,0,0);
    
}
