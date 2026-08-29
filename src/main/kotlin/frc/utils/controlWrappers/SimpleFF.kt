package frc.utils.controlWrappers

import edu.wpi.first.math.controller.SimpleMotorFeedforward

/*
 * wrapper for SimpleMotorFeedforward that allows setting gains from a record
 */
class SimpleFF : SimpleMotorFeedforward {
    constructor(kS: Double, kV: Double, kA: Double) : super(kS, kV, kA)
    constructor(kS: Double, kV: Double, kA: Double, dt: Double) : super(kS, kV, kA, dt)
    constructor(gains: PIDGains.SimpleFF) : super(gains.kS, gains.kV, gains.kA)

    fun setGains(gains: PIDGains.SimpleFF) {
        setKs(gains.kS)
        setKv(gains.kV)
        setKa(gains.kA)
    }
}
