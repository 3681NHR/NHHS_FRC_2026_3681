package frc.robot.constants

import edu.wpi.first.units.Units.Celsius
import edu.wpi.first.units.Units.RotationsPerSecond
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Temperature
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.utils.controlWrappers.PIDGains
import org.littletonrobotics.junction.Logger

object LauncherConstants {

    const val LAUNCHER_MOTOR_ID: Int = 40

    @JvmField val LAUNCHER_ID_GAINS: PIDGains.SimpleFF = PIDGains.SimpleFF(0.0, 0.13157, 2.3)

    @JvmField val LAUNCHER_FF_GAINS: PIDGains.SimpleFF = LAUNCHER_ID_GAINS
    @JvmField val LAUNCHER_PID_GAINS: PIDGains.PID = PIDGains.PID(0.4, 0.0, 0.0)

    @JvmField val LAUNCHER_SETPOINT_TOLERANCE: AngularVelocity = RotationsPerSecond.of(10.0)
    @JvmField val LAUNCHER_MAX_TEMP: Temperature = Celsius.of(40.0)

    @JvmField val LAUNCHER_SYSID_CONFIG: SysIdRoutine.Config = SysIdRoutine.Config(
        Volts.per(Second).of(1.0),
        Volts.of(5.0),
        Seconds.of(5.0),
        { state -> Logger.recordOutput("Launcher/SysIdTestState", state.toString()) }
    )
}
