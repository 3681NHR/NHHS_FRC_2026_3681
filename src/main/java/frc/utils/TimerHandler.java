package frc.utils;

import static edu.wpi.first.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.Constants;

/**
 * handles a timer for teleop and auto, sends all data to log/NT
 */
public class TimerHandler {

    private static double autoStart = 0.0;
    private static double teleopStart = 0.0;

    public static void updateTeleop(){
        Logger.recordOutput("Utils/time/teleop/UpTime"       , Timer.getFPGATimestamp()-teleopStart);
        Logger.recordOutput("Utils/time/teleop/RemainingTime", Math.max(0, Constants.TELEOP_TIME.in(Seconds)-(Timer.getFPGATimestamp()-teleopStart)));
        Logger.recordOutput("Utils/time/RemainingTime", Math.max(0, Constants.TELEOP_TIME.in(Seconds)-(Timer.getFPGATimestamp()-teleopStart)));
    }
    public static void updateAuto(){
        Logger.recordOutput("Utils/time/auto/UpTime"       , Timer.getFPGATimestamp()-autoStart);
        Logger.recordOutput("Utils/time/auto/RemainingTime", Math.max(0, Constants.AUTO_TIME.in(Seconds)-(Timer.getFPGATimestamp()-autoStart)));
        Logger.recordOutput("Utils/time/RemainingTime", Math.max(0, Constants.AUTO_TIME.in(Seconds)-(Timer.getFPGATimestamp()-autoStart)));
    }
    public static void initTeleop(){
        teleopStart = Timer.getFPGATimestamp();
        Logger.recordOutput("Utils/time/teleop/StartTime", teleopStart);
    }
    public static void initAuto(){
        autoStart = Timer.getFPGATimestamp();
        Logger.recordOutput("Utils/time/auto/StartTime", autoStart);
    }
    public static void init(){
        Logger.recordOutput("Utils/time/upTime", 0.0);
        Logger.recordOutput("Utils/time/RemainingTime", 0.0);

        Logger.recordOutput("Utils/time/auto/StartTime"    , 0.0);
        Logger.recordOutput("Utils/time/auto/UpTime"       , 0.0);
        Logger.recordOutput("Utils/time/auto/RemainingTime", 0.0);
        Logger.recordOutput("Utils/time/auto/TotalTime"    , Constants.AUTO_TIME);
        
        Logger.recordOutput("Utils/time/teleop/StartTime"    , 0.0);
        Logger.recordOutput("Utils/time/teleop/UpTime"       , 0.0);
        Logger.recordOutput("Utils/time/teleop/RemainingTime", 0.0);
        Logger.recordOutput("Utils/time/teleop/TotalTime"    , Constants.TELEOP_TIME);
    }
    public static void update(){
        Logger.recordOutput("Utils/time/upTime", Timer.getFPGATimestamp());
    } 
    public static double getTeleopRemaining(){
        return Math.max(0, Constants.TELEOP_TIME.in(Seconds)-(Timer.getFPGATimestamp()-teleopStart));
    }
    public static double getAutoRemaining(){
        return Math.max(0, Constants.AUTO_TIME.in(Seconds)-(Timer.getFPGATimestamp()-autoStart));
    }
    
}
