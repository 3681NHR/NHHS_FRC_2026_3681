package frc.utils.controlWrappers

import edu.wpi.first.math.controller.ProfiledPIDController
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints

/*
 * wrapper for ProfiledPIDController that allows setting gains from a record
 */
class ProfiledPID : ProfiledPIDController {
    constructor(kP: Double, kI: Double, kD: Double, constraints: Constraints) : super(kP, kI, kD, constraints)
    constructor(kP: Double, kI: Double, kD: Double, constraints: Constraints, period: Double) : super(kP, kI, kD, constraints, period)
    constructor(gains: PIDGains.ProfiledPID) : super(gains.kP, gains.kI, gains.kD, Constraints(gains.maxSpeed, gains.maxAccel))

    fun setSpeed(maxSpeed: Double) {
        setConstraints(Constraints(maxSpeed, super.getConstraints().maxAcceleration))
    }

    fun setAccel(maxAccel: Double) {
        setConstraints(Constraints(super.getConstraints().maxVelocity, maxAccel))
    }

    fun getMaxSpeed(): Double {
        return super.getConstraints().maxVelocity
    }

    fun getMaxAccel(): Double {
        return super.getConstraints().maxAcceleration
    }

    fun setGains(gains: PIDGains.ProfiledPID) {
        setPID(gains.kP, gains.kI, gains.kD)
        setConstraints(Constraints(gains.maxSpeed, gains.maxAccel))
    }
}
