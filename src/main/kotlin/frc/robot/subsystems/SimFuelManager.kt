package frc.robot.subsystems

import org.ironmaple.simulation.IntakeSimulation

class SimFuelManager private constructor() {

    @JvmField
    var capacity: Int = 0

    @JvmField
    var intake: IntakeSimulation? = null

    companion object {
        @Volatile
        private var instance: SimFuelManager? = null

        @Synchronized
        @JvmStatic
        fun getInstance(): SimFuelManager {
            if (instance == null) {
                instance = SimFuelManager()
            }
            return instance!!
        }
    }
}
