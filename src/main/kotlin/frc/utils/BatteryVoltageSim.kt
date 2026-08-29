package frc.utils

import edu.wpi.first.wpilibj.simulation.BatterySim
import edu.wpi.first.wpilibj.simulation.RoboRioSim
import java.util.function.DoubleSupplier

/**
 * singleton class to manage current draw for all simulated parts of the robot
 */
class BatteryVoltageSim private constructor() {

    private var voltage: Double = 11.5 // unloaded voltage
    private var nominalcurrent: Double = 20.0 // default current draw
    private var current: Double = 0.0

    private val currentSources: MutableList<DoubleSupplier> = ArrayList()

    init {
        currentSources.add(DoubleSupplier { nominalcurrent })
    }

    fun addCurrentSource(currentSource: DoubleSupplier) {
        currentSources.add(currentSource)
    }

    fun calculateVoltage(): Double {
        current = 0.0
        for (currentSource in currentSources) {
            current += currentSource.asDouble
        }
        val loadVoltage = BatterySim.calculateLoadedBatteryVoltage(voltage, 0.015, current)
        RoboRioSim.setVInVoltage(voltage)
        return loadVoltage
    }

    companion object {
        @Volatile
        private var instance: BatteryVoltageSim? = null

        @JvmStatic
        @Synchronized
        fun getInstance(): BatteryVoltageSim {
            if (instance == null) {
                instance = BatteryVoltageSim()
            }
            return instance!!
        }
    }
}
