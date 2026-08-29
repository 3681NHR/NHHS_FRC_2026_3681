package frc.utils.rumble

import edu.wpi.first.wpilibj.GenericHID.RumbleType
import edu.wpi.first.wpilibj.XboxController
import org.littletonrobotics.junction.Logger
import java.util.ArrayList
import java.util.Collection

class RumbleHandler {
    private var controller: XboxController? = null
    private var port: Double = 0.0

    private var following: Boolean = false
    private var lead: RumbleHandler? = null

    private var queue: ArrayList<Rumble> = ArrayList()

    constructor(controller: XboxController) {
        this.controller = controller
        this.port = controller.port.toDouble()
    }

    constructor(lead: RumbleHandler) {
        this.following = true
        this.lead = lead
    }

    /**
     * clear rumble queue, effectively stopping all rumble
     */
    fun clearQue() {
        queue.clear()
    }

    fun addToQue(a: Array<Rumble>) {
        for (b in a) {
            queue.add(b)
        }
    }

    fun addToQue(a: Collection<Rumble>) {
        for (b in a) {
            queue.add(b)
        }
    }

    fun addToQue(a: Rumble) {
        queue.add(a)
    }

    fun overrideQue(a: Rumble) {
        clearQue()
        queue.add(a)
    }

    fun overrideQue(a: Collection<Rumble>) {
        clearQue()
        for (b in a) {
            queue.add(b)
        }
    }

    fun overrideQue(a: Array<Rumble>) {
        clearQue()
        for (b in a) {
            queue.add(b)
        }
    }

    fun update(loopTime: Double) {
        if (following) {
            queue = lead!!.queue
        }
        if (queue.size > 0) {
            queue[0].time -= loopTime
        }
        var i = 0
        while (i < queue.size) {
            if (queue[i].time <= 0) {
                queue.removeAt(i)
                // preserve original Java for-loop bug where i increments even after remove,
                // which would skip next element; replicate exactly with manual increment
                // Original: for (int i=0; i < queue.size(); i++) { if(queue.get(i).time <= 0){ queue.remove(i); } }
                // To replicate skip behavior, we increment i even after removal — but that would skip.
                // Instead replicate the buggy for-loop literally:
            }
            i++
        }
        if (queue.size >= 1) {
            controller!!.setRumble(RumbleType.kLeftRumble, queue[0].powL)
            controller!!.setRumble(RumbleType.kRightRumble, queue[0].powR)
            Logger.recordOutput("Utils/haptics/rumble: $port/current Strength left", queue[0].powL)
            Logger.recordOutput("Utils/haptics/rumble: $port/current Strength right", queue[0].powR)
            Logger.recordOutput("Utils/haptics/rumble: $port/queue", getPows())
        } else {
            controller!!.setRumble(RumbleType.kBothRumble, 0.0)
            Logger.recordOutput("Utils/haptics/rumble: $port/current Strength left", 0.0)
            Logger.recordOutput("Utils/haptics/rumble: $port/current Strength right", 0.0)
            Logger.recordOutput("Utils/haptics/rumble: $port/queue", Array(0) { DoubleArray(0) })
        }
        Logger.recordOutput("Utils/haptics/rumble: $port/following", following)

        if (following) {
            Logger.recordOutput("Utils/haptics/rumble: $port/following rumble", lead!!.port)
        }
    }

    private fun getPows(): Array<DoubleArray> {
        val pows = Array(queue.size) { DoubleArray(3) }
        for (i in queue.indices) {
            val d = queue[i]
            pows[i][0] = d.powR
            pows[i][1] = d.powL
            pows[i][2] = d.time
        }
        return pows
    }
}
