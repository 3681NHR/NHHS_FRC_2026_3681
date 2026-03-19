package frc.robot.constants;

import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.utils.controlWrappers.PIDGains;

public final class LauncherConstants {
    
    public static final int LAUNCHER_MOTOR_ID = 40;

    public static final PIDGains.SimpleFF LAUNCHER_ID_GAINS = new PIDGains.SimpleFF(0.0,0.13157,2.3);//gains from sysid for state space model

    public static final PIDGains.SimpleFF LAUNCHER_FF_GAINS = LAUNCHER_ID_GAINS;
    public static final PIDGains.PID LAUNCHER_PID_GAINS = new PIDGains.PID(0.4,0,0.0);

    public static final AngularVelocity LAUNCHER_SETPOINT_TOLERANCE = RotationsPerSecond.of(10);
    public static final Temperature LAUNCHER_MAX_TEMP = Celsius.of(40);

    public static final SysIdRoutine.Config LAUNCHER_SYSID_CONFIG = new SysIdRoutine.Config(
        Volts.per(Second).of(1.0), 
        Volts.of(5.0), 
        Seconds.of(5),
        (state) -> Logger.recordOutput("Launcher/SysIdTestState", state.toString())
    );
}
