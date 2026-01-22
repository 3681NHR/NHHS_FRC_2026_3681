package frc.robot.constants;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.controlWrappers.PIDGains;

public final class LauncherConstants {
    
    public static final int LAUNCHER_MOTOR_ID = -1;

    public static final PIDGains.SimpleFF LAUNCHER_ID_GAINS = new PIDGains.SimpleFF(0.0,3,0.01);//gains from sysid for state space model

    public static final PIDGains.SimpleFF LAUNCHER_FF_GAINS = LAUNCHER_ID_GAINS;
    public static final PIDGains.PID LAUNCHER_PID_GAINS = new PIDGains.PID(3.0,0,1.0);

    public static final double LAUNCHER_SETPOINT_TOLERANCE = 100;//rpm

    public static final SysIdRoutine.Config LAUNCHER_SYSID_CONFIG = new SysIdRoutine.Config(
        Volts.per(Second).of(1.0), 
        Volts.of(5.0), 
        Seconds.of(5),
        (state) -> Logger.recordOutput("Launcher/SysIdTestState", state.toString())
    );
}
