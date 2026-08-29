package frc.utils.rumble

class Rumble {
    @JvmField var powR: Double
    @JvmField var powL: Double
    @JvmField var time: Double

    constructor(time: Double, pow: Double) {
        this.powR = pow
        this.powL = pow
        this.time = time
    }

    constructor(time: Double, powR: Double, powL: Double) {
        this.powR = powR
        this.powL = powL
        this.time = time
    }
}
