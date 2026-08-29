package frc.robot.constants

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.utils.controlWrappers.PIDGains.ProfiledPID
import frc.utils.controlWrappers.PIDGains.SimpleFF
import org.littletonrobotics.junction.Logger

object TurretConstants {

    const val TURRET_MOTOR_ID: Int = 50
    const val TURRET_ENCODER_1_ID: Int = 31
    const val TURRET_ENCODER_2_ID: Int = 32

    const val TURRET_MAIN_GEAR_TEETH: Double = 200.0
    const val TURRET_ENCODER_1_GEAR_TEETH: Double = 35.0
    const val TURRET_ENCODER_2_GEAR_TEETH: Double = 34.0
    const val TURRET_MOTOR_GEAR_TEETH: Double = 20.0

    @JvmField val SLOPE: Double = (TURRET_ENCODER_2_GEAR_TEETH * TURRET_ENCODER_1_GEAR_TEETH) /
        ((TURRET_ENCODER_1_GEAR_TEETH - TURRET_ENCODER_2_GEAR_TEETH) * TURRET_MAIN_GEAR_TEETH)

    @JvmField val TURRET_CURRENT_LIM: Current = Amps.of(20.0)
    const val TURRET_MOTOR_INVERT: Boolean = true

    @JvmField val TURRET_ANGLE_FORWARD_LIM: Angle = Degrees.of(240.0)
    @JvmField val TURRET_ANGLE_REVERSE_LIM: Angle = Degrees.of(-240.0)

    @JvmField val TURRET_ID_GAINS: SimpleFF = SimpleFF(0.3, 1.25, 0.00001)

    @JvmField val TURRET_FF_GAINS: SimpleFF = SimpleFF(0.4, 1.35, 0.0).makeTunable("Tuning/Turret/FF")
    @JvmField val TURRET_PID_GAINS: ProfiledPID = ProfiledPID(7.0, 1.0, 0.3, 1.5, 10.0).makeTunable("Tuning/Turret/PID")
    const val TURRET_THETA_COMP_FACTOR: Double = -0.08

    @JvmField val TURRET_SETPOINT_TOLERANCE: Angle = Degrees.of(5.0)

    @JvmField val HUB_RADIUS: Distance = Inches.of(45.7 / 2)
    @JvmField val PASS_RADIUS: Distance = Meters.of(0.75)

    @JvmField val RED_HUB: Translation2d = Translation2d(11.915, 4.034)
    @JvmField val RED_PASS: Array<Translation2d> = arrayOf(
        Translation2d(14.0, 6.3),
        Translation2d(14.0, 2.2)
    )

    @JvmField val BLUE_HUB: Translation2d = Translation2d(4.625, 4.034)
    @JvmField val BLUE_PASS: Array<Translation2d> = arrayOf(
        Translation2d(2.5, 6.3),
        Translation2d(2.5, 2.2)
    )

    @JvmField val TURRET_OFFSET: Translation3d = Translation3d(-0.146050, 0.152400, 0.299237)
    @JvmField val HOOD_TO_TURRET_OFFSET: Translation3d = Translation3d(0.090695, -0.00681, 0.140578)

    @JvmField val TURRET_SYSID_CONFIG: SysIdRoutine.Config = SysIdRoutine.Config(
        Volts.per(Second).of(1.0),
        Volts.of(5.0),
        Seconds.of(5.0),
        { state -> Logger.recordOutput("Turret/SysIdTestState", state.toString()) }
    )
}
