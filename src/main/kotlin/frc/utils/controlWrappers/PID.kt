package frc.utils.controlWrappers

import edu.wpi.first.math.controller.PIDController

/*
 * wrapper for PIDController that allows setting gains from a record
 */
class PID : PIDController {
    constructor(kP: Double, kI: Double, kD: Double) : super(kP, kI, kD)
    constructor(kP: Double, kI: Double, kD: Double, period: Double) : super(kP, kI, kD, period)
    constructor(gains: PIDGains.PID) : super(gains.kP, gains.kI, gains.kD)
}
