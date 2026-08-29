package frc.robot.subsystems.fuelVision

import org.littletonrobotics.junction.AutoLog

interface FuelVisionIO {
    fun updateInputs(inputs: FuelVisionIOInputs) {}

    @AutoLog
    open class FuelVisionIOInputs {
        @JvmField var observations: Array<FuelObservation> = arrayOf(
            FuelObservation(RadialPos2d(0.0, 0.0), ScreenSize2d(0.0, 0.0), 0.0, 0.0, 0.0)
        )
        @JvmField var connected: Boolean = false
        @JvmField var timestamp: Double = 0.0
    }
}
