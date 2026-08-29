package frc.utils

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkBase
import edu.wpi.first.units.Units.Seconds
import edu.wpi.first.wpilibj.Timer
import org.ironmaple.simulation.SimulatedArena
import java.util.function.Supplier

object SparkUtil {
    @JvmField var sparkStickyFault: Boolean = false

    @JvmStatic
    fun tryUntilOk(spark: SparkBase, maxAttempts: Int, command: Supplier<REVLibError>) {
        for (i in 0 until maxAttempts) {
            val error = command.get()
            if (error == REVLibError.kOk) {
                break
            } else {
                sparkStickyFault = true
            }
        }
    }

    @JvmStatic
    fun getSimulationOdometryTimeStamps(): DoubleArray {
        val odometryTimeStamps = DoubleArray(SimulatedArena.getSimulationSubTicksIn1Period())
        for (i in odometryTimeStamps.indices) {
            odometryTimeStamps[i] = Timer.getFPGATimestamp() - 0.02 + i * SimulatedArena.getSimulationDt().`in`(Seconds)
        }
        return odometryTimeStamps
    }
}
