package frc.robot.constants

import edu.wpi.first.math.geometry.Translation3d
import edu.wpi.first.units.Units.Amps
import edu.wpi.first.units.Units.Degrees
import edu.wpi.first.units.Units.Volts
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Current
import edu.wpi.first.units.measure.Voltage
import frc.utils.controlWrappers.PIDGains

object IntakeConstants {

    const val INTAKE_ROLLER_MOTOR_ID: Int = 42

    const val INTAKE_PIVOT_MOTOR_ID: Int = 52
    const val INTAKE_PIVOT_ENCODER_ID: Int = 34

    const val INTAKE_ROLLER_INVERTED: Boolean = false
    @JvmField val INTAKE_ROLLER_CURRENT_LIM: Current = Amps.of(30.0)

    const val INTAKE_PIVOT_INVERTED: Boolean = false
    @JvmField val INTAKE_PIVOT_CURRENT_LIM: Current = Amps.of(40.0)

    @JvmField val INTAKE_RUN_VOLTAGE: Voltage = Volts.of(11.0)
    @JvmField val INTAKE_EJECT_VOLTAGE: Voltage = Volts.of(-10.0)

    @JvmField val INTAKE_STOWED_ANGLE: Angle = Degrees.of(125.0)
    @JvmField val INTAKE_DEPLOYED_ANGLE: Angle = Degrees.of(0.0)

    @JvmField val INTAKE_PIVOT_MIN_ANGLE: Angle = Degrees.of(-5.0)
    @JvmField val INTAKE_PIVOT_MAX_ANGLE: Angle = Degrees.of(130.0)

    @JvmField val INTAKE_PIVOT_TOLERANCE: Angle = Degrees.of(5.0)

    @JvmField val INTAKE_PIVOT_ID_GAINS: PIDGains.GravityFF = PIDGains.GravityFF(0.0, 0.3, 0.7, 0.001)

    @JvmField val INTAKE_PIVOT_FF_GAINS: PIDGains.GravityFF =
        PIDGains.GravityFF(0.0, 0.3, 0.7, 0.0).makeTunable("Tuning/Intake/Pivot/FF")
    @JvmField val INTAKE_PIVOT_PID_GAINS: PIDGains.ProfiledPID =
        PIDGains.ProfiledPID(0.3, 0.00067, 0.2, 7.0, 15.0).makeTunable("Tuning/Intake/Pivot/PID")

    @JvmField val INTAKE_OFFSET: Translation3d = Translation3d(0.292100, 0.0, 0.191)
}
