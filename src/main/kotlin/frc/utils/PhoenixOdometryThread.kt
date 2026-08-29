package frc.utils

import com.ctre.phoenix6.BaseStatusSignal
import com.ctre.phoenix6.StatusSignal
import edu.wpi.first.units.Units.Hertz
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.RobotController
import frc.robot.constants.DriveConstants
import frc.robot.subsystems.swerve.Drive
import java.util.ArrayList
import java.util.Queue
import java.util.concurrent.ArrayBlockingQueue
import java.util.concurrent.locks.ReentrantLock
import java.util.function.DoubleSupplier

class PhoenixOdometryThread private constructor() : Thread() {
    private val signalsLock = ReentrantLock()
    private var phoenixSignals: Array<BaseStatusSignal> = emptyArray()
    private val genericSignals: MutableList<DoubleSupplier> = ArrayList()
    private val phoenixQueues: MutableList<Queue<Double>> = ArrayList()
    private val genericQueues: MutableList<Queue<Double>> = ArrayList()
    private val timestampQueues: MutableList<Queue<Double>> = ArrayList()

    init {
        name = "PhoenixOdometryThread"
        isDaemon = true
    }

    override fun start() {
        if (timestampQueues.size > 0) {
            super.start()
        }
    }

    fun registerSignal(signal: StatusSignal<Angle>): Queue<Double> {
        val queue: Queue<Double> = ArrayBlockingQueue(20)
        signalsLock.lock()
        Drive.odometryLock.lock()
        try {
            val newSignals = arrayOfNulls<BaseStatusSignal>(phoenixSignals.size + 1)
            System.arraycopy(phoenixSignals, 0, newSignals, 0, phoenixSignals.size)
            newSignals[phoenixSignals.size] = signal
            @Suppress("UNCHECKED_CAST")
            phoenixSignals = newSignals as Array<BaseStatusSignal>
            phoenixQueues.add(queue)
        } finally {
            signalsLock.unlock()
            Drive.odometryLock.unlock()
        }
        return queue
    }

    fun registerSignal(signal: DoubleSupplier): Queue<Double> {
        val queue: Queue<Double> = ArrayBlockingQueue(20)
        signalsLock.lock()
        Drive.odometryLock.lock()
        try {
            genericSignals.add(signal)
            genericQueues.add(queue)
        } finally {
            signalsLock.unlock()
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

    override fun run() {
        while (true) {
            signalsLock.lock()
            try {
                if (isCANFD && phoenixSignals.isNotEmpty()) {
                    BaseStatusSignal.waitForAll(2.0 / DriveConstants.ODOMETRY_FREQ.`in`(Hertz), *phoenixSignals)
                } else {
                    sleep((1000.0 / DriveConstants.ODOMETRY_FREQ.`in`(Hertz)).toLong())
                    if (phoenixSignals.isNotEmpty()) BaseStatusSignal.refreshAll(*phoenixSignals)
                }
            } catch (e: InterruptedException) {
                e.printStackTrace()
            } finally {
                signalsLock.unlock()
            }

            Drive.odometryLock.lock()
            try {
                var timestamp = RobotController.getFPGATime() / 1e6
                var totalLatency = 0.0
                for (signal in phoenixSignals) {
                    totalLatency += signal.timestamp.latency
                }
                if (phoenixSignals.isNotEmpty()) {
                    timestamp -= totalLatency / phoenixSignals.size
                }

                for (i in phoenixSignals.indices) {
                    phoenixQueues[i].offer(phoenixSignals[i].valueAsDouble)
                }
                for (i in genericSignals.indices) {
                    genericQueues[i].offer(genericSignals[i].asDouble)
                }
                for (i in timestampQueues.indices) {
                    timestampQueues[i].offer(timestamp)
                }
            } finally {
                Drive.odometryLock.unlock()
            }
        }
    }

    companion object {
        @JvmStatic var isCANFD: Boolean = false
            private set

        private var instance: PhoenixOdometryThread? = null

        @JvmStatic
        fun getInstance(): PhoenixOdometryThread {
            if (instance == null) {
                instance = PhoenixOdometryThread()
            }
            return instance!!
        }
    }
}
