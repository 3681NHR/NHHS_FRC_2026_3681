package frc.robot.constants

import com.pathplanner.lib.config.ModuleConfig
import com.pathplanner.lib.config.RobotConfig
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.system.plant.DCMotor
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.Units.Inches
import edu.wpi.first.units.Units.KilogramSquareMeters
import edu.wpi.first.units.Units.Kilograms
import edu.wpi.first.units.Units.Meters
import edu.wpi.first.units.Units.MetersPerSecond
import edu.wpi.first.units.Units.MetersPerSecondPerSecond
import edu.wpi.first.units.Units.Radians
import edu.wpi.first.units.Units.RadiansPerSecond
import edu.wpi.first.units.Units.RadiansPerSecondPerSecond
import edu.wpi.first.units.Units.Second
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.VoltageUnit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularAcceleration
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Frequency
import edu.wpi.first.units.measure.LinearAcceleration
import edu.wpi.first.units.measure.LinearVelocity
import edu.wpi.first.units.measure.Mass
import edu.wpi.first.units.measure.MomentOfInertia
import edu.wpi.first.units.measure.Time
import edu.wpi.first.units.measure.Velocity
import edu.wpi.first.units.measure.Voltage
import frc.utils.controlWrappers.PIDGains

object DriveConstants {
    const val ANGULAR_VELOCITY_COEFFICIENT: Double = 0.02

    @JvmField val AUTO_ALIGN_ANGLE_MAX_OFFSET: Angle = Degrees.of(0.05)
    @JvmField val AUTO_ALIGN_POS_MAX_OFFSET: Distance = Meters.of(0.01)

    @JvmField var USE_VISION: Boolean = true

    @JvmField val MAX_SPEED_PP: LinearVelocity = MetersPerSecond.of(4.765)
    @JvmField val MAX_ACCEL_PP: LinearAcceleration = MetersPerSecondPerSecond.of(2.0)
    @JvmField val MAX_ANGLE_SPEED_PP: AngularVelocity = RadiansPerSecond.of(8.461)
    @JvmField val MAX_ANGLE_ACCEL_PP: AngularAcceleration =
        RadiansPerSecondPerSecond.of(MAX_ANGLE_SPEED_PP.`in`(RadiansPerSecond) * 3)

    @JvmField val AUTO_ANGLE_PID: PIDGains.PID = PIDGains.PID(5.0, 0.0, 0.5)
    @JvmField val TRANS_PID: PIDGains.PID = PIDGains.PID(4.0, 0.0, 0.3)
    @JvmField val TRANS_PID_SIM: PIDGains.PID = PIDGains.PID(4.0, 0.0, 0.3)

    @JvmField val ANGLE_MAX_VELOCITY: AngularVelocity = MAX_ANGLE_SPEED_PP

    @JvmField val ODOMETRY_FREQ: Frequency = Hertz.of(100.0)
    @JvmField val WIDTH: Distance = Inches.of(22.0)
    @JvmField val LENGTH: Distance = Inches.of(25.0)
    @JvmField val RADIUS: Distance = Meters.of(Math.hypot(WIDTH.`in`(Meters) / 2.0, LENGTH.`in`(Meters) / 2.0))
    @JvmField val MODULE_POSITIONS: Array<Translation2d> = arrayOf(
        Translation2d(WIDTH.`in`(Meters) / 2.0, LENGTH.`in`(Meters) / 2.0),
        Translation2d(WIDTH.`in`(Meters) / 2.0, -LENGTH.`in`(Meters) / 2.0),
        Translation2d(-WIDTH.`in`(Meters) / 2.0, LENGTH.`in`(Meters) / 2.0),
        Translation2d(-WIDTH.`in`(Meters) / 2.0, -LENGTH.`in`(Meters) / 2.0)
    )

    @JvmField val MASS: Mass = Kilograms.of(60.0)
    @JvmField val MOI: MomentOfInertia = KilogramSquareMeters.of(6.0)
    const val COF: Double = 2.31421199
    @JvmField val PP_CONFIG: RobotConfig = RobotConfig(
        MASS,
        MOI,
        ModuleConfig(
            Module.WHEEL_RAD.`in`(Meters),
            MAX_SPEED_PP.`in`(MetersPerSecond),
            COF,
            Module.DRIVE_GEARBOX.withReduction(Module.DRIVE_REDUCTION),
            Module.DRIVE_MAX_CURRENT.`in`(Amps),
            1
        ),
        *MODULE_POSITIONS
    )

    @JvmField val DRIVE_SYSID_VSTEP: Voltage = Volts.of(2.0)
    @JvmField val DRIVE_SYSID_VRAMP: Velocity<VoltageUnit> = Volts.of(0.5).per(Second)
    @JvmField val DRIVE_SYSID_TIMEOUT: Time = Seconds.of(5.0)

    @JvmField val TURN_SYSID_VSTEP: Voltage = Volts.of(7.0)
    @JvmField val TURN_SYSID_VRAMP: Velocity<VoltageUnit> = Volts.of(1.0).per(Second)
    @JvmField val TURN_SYSID_TIMEOUT: Time = Seconds.of(10.0)

    const val GYRO_ID: Int = 30

    object Module {
        const val FL_DRIVE_ID: Int = 11
        const val FR_DRIVE_ID: Int = 12
        const val BL_DRIVE_ID: Int = 13
        const val BR_DRIVE_ID: Int = 14

        const val FL_TURN_ID: Int = 21
        const val FR_TURN_ID: Int = 22
        const val BL_TURN_ID: Int = 23
        const val BR_TURN_ID: Int = 24

        const val DRIVE_INVERT: Boolean = false
        @JvmField val DRIVE_MAX_CURRENT: Current = Amps.of(50.0)
        @JvmField val DRIVE_SLIP_CURRENT: Current = Amps.of(50.0)
        const val DRIVE_REDUCTION: Double = 6.75
        @JvmField val WHEEL_RAD: Distance = Inches.of(2.05)
        @JvmField val DRIVE_GEARBOX: DCMotor = DCMotor.getNEO(1)

        @JvmField val DRIVE_PID: PIDGains.PID = PIDGains.PID(0.25, 0.0, 0.0).makeTunable("Tuning/Drive/PID")
        @JvmField val DRIVE_FF: PIDGains.SimpleFF = PIDGains.SimpleFF(0.16, 0.76217, 0.016315).makeTunable("Tuning/Drive/FF")
        @JvmField val DRIVE_PID_SIM: PIDGains.PID = PIDGains.PID(0.01, 0.0, 0.0)
        @JvmField val DRIVE_FF_SIM: PIDGains.SimpleFF = PIDGains.SimpleFF(0.11, 0.13, 0.1)

        const val TURN_INVERT: Boolean = true
        @JvmField val TURN_CURRENT_LIM: Current = Amps.of(20.0)
        const val TURN_REDUCTION: Double = 21.428
        @JvmField val TURN_GEARBOX: DCMotor = DCMotor.getNEO(1)

        const val TURN_ENCODER_INVERT: Boolean = true
        @JvmField val TURN_ENCODER_POS_FACTOR: Double = 2 * Math.PI
        @JvmField val TURN_ENCODER_VEL_FACTOR: Double = (2 * Math.PI) / 60.0

        @JvmField val TURN_PID: PIDGains.ProfiledPID = PIDGains.ProfiledPID(7.5, 0.0, 0.0, Math.PI * 8, Math.PI * 80)
        @JvmField val TURN_FF: PIDGains.SimpleFF = PIDGains.SimpleFF(0.125, 0.0, 0.0)
        @JvmField val TURN_PID_SIM: PIDGains.ProfiledPID = PIDGains.ProfiledPID(12.5, 0.0, 0.5, Math.PI * 8, Math.PI * 80)
        @JvmField val TURN_FF_SIM: PIDGains.SimpleFF = PIDGains.SimpleFF(0.015, 0.0, 0.0)

        @JvmField val TURN_MIN_POS: Angle = Radians.of(0.0)
        @JvmField val TURN_MAX_POS: Angle = Radians.of(2 * Math.PI)

        const val DRIVE_OFFSET_VEL_FACTOR: Double = 0.0
    }
}
