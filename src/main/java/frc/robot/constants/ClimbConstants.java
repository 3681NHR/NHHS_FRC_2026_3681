package frc.robot.constants;

import frc.utils.controlWrappers.PIDGains;

public class ClimbConstants {
    public static final int MOTOR_ID = 5;
    public static final boolean INVERTED = false;
    public static final double EXTEND_POSITION = 1.0;
    public static final PIDGains.ProfiledPID CLIMB_PID_GAINS = new PIDGains.ProfiledPID(0, 0, 0, 0, 0).makeTunable("IO/Climber/PID");
    public static final PIDGains.GravityFF FF = new PIDGains.GravityFF(0, 0, 0, 0).makeTunable("IO/Climber/FF");
}
