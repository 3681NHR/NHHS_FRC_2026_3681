package frc.utils

import com.revrobotics.REVLibError
import com.revrobotics.spark.SparkBase
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.wpilibj.Notifier
import edu.wpi.first.wpilibj.RobotController
import frc.robot.constants.DriveConstants
import frc.robot.subsystems.swerve.Drive
import java.util.ArrayList
import java.util.Queue
import java.util.concurrent.ArrayBlockingQueue
import java.util.function.DoubleSupplier

class SparkOdometryThread private constructor() {
    private val sparks: MutableList<SparkBase> = ArrayList()
    private val sparkSignals: MutableList<DoubleSupplier> = ArrayList()
    private val genericSignals: MutableList<DoubleSupplier> = ArrayList()
    private val sparkQueues: MutableList<Queue<Double>> = ArrayList()
    private val genericQueues: MutableList<Queue<Double>> = ArrayList()
    private val timestampQueues: MutableList<Queue<Double>> = ArrayList()

    private val notifier = Notifier(this::run)

    init {
        notifier.setName("OdometryThread")
    }

    fun start() {
        if (timestampQueues.size > 0) {
            notifier.startPeriodic(1.0 / DriveConstants.ODOMETRY_FREQ.`in`(Hertz))
        }
    }

    fun registerSignal(spark: SparkBase, signal: DoubleSupplier): Queue<Double> {
        val queue: Queue<Double> = ArrayBlockingQueue(20)
        Drive.odometryLock.lock()
        try {
            sparks.add(spark)
            sparkSignals.add(signal)
            sparkQueues.add(queue)
        } finally {
            Drive.odometryLock.unlock()
        }
        return queue
    }

    fun registerSignal(signal: DoubleSupplier): Queue<Double> {
        val queue: Queue<Double> = ArrayBlockingQueue(20)
        Drive.odometryLock.lock()
        try {
            genericSignals.add(signal)
            genericQueues.add(queue)
        } finally {
            Drive.odometryLock.unlock()
        }
        return queue
    }

    fun makeTimestampQueue(): Queue<Double> {
        val queue: Queue<Double> = ArrayBlockingQueue(20)
        Drive.odometryLock.lock()
        try {
            timestampQueues.add(queue)
        } finally {
            Drive.odometryLock.unlock()
        }
        return queue
    }

    private fun run() {
        Drive.odometryLock.lock()
        try {
            val timestamp = RobotController.getFPGATime() / 1e6

            val sparkValues = DoubleArray(sparkSignals.size)
            var isValid = true
            for (i in sparkSignals.indices) {
                sparkValues[i] = sparkSignals[i].asDouble
                if (sparks[i].lastError != REVLibError.kOk) {
                    isValid = false
                }
            }

            if (isValid) {
                for (i in sparkSignals.indices) {
                    sparkQueues[i].offer(sparkValues[i])
                }
                for (i in genericSignals.indices) {
                    genericQueues[i].offer(genericSignals[i].asDouble)
                }
                for (i in timestampQueues.indices) {
                    timestampQueues[i].offer(timestamp)
                }
            }
        } finally {
            Drive.odometryLock.unlock()
        }
    }

    companion object {
        private var instance: SparkOdometryThread? = null

        @JvmStatic
        fun getInstance(): SparkOdometryThread {
            if (instance == null) {
                instance = SparkOdometryThread()
            }
            return instance!!
        }
    }
}
