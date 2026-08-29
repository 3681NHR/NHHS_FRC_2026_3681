package frc.utils.controlWrappers

import edu.wpi.first.math.controller.ArmFeedforward

/*
 * wrapper for ArmFeedforward that allows setting gains from a record
 */
class ArmFF : ArmFeedforward {
    constructor(kS: Double, kG: Double, kV: Double, kA: Double) : super(kS, kG, kV, kA)
    constructor(kS: Double, kG: Double, kV: Double, kA: Double, dt: Double) : super(kS, kG, kV, kA, dt)
    constructor(gains: PIDGains.GravityFF) : super(gains.kS, gains.kG, gains.kV, gains.kA)
}
