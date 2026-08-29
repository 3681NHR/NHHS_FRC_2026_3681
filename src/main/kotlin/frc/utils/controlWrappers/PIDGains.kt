package frc.utils.controlWrappers

import java.util.ArrayList
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber
import edu.wpi.first.wpilibj.DriverStation
import frc.utils.PIDTuner

class PIDGains {
    open class Gains {
        var key: String? = null
        var apply: LoggedNetworkBoolean? = null
        var applyCallback: ArrayList<Runnable> = ArrayList()

        open fun update() {
        }

        open fun makeTunable(key: String): Gains {
            this.key = key
            apply = LoggedNetworkBoolean("$key/apply", false)
            PIDTuner.tunableGains.add(this)
            return this
        }

        /**
         * set a callback for when applied,
         * @param onApply called when the gains are set using the live tuning
         * @return
         */
        fun withCallback(onApply: Runnable): Gains {
            applyCallback.add(onApply)
            return this
        }
    }

    /**
     * simple class for PID gains
     */
    class PID(
        @JvmField var kP: Double,
        @JvmField var kI: Double,
        @JvmField var kD: Double
    ) : Gains() {
        var setkP: LoggedNetworkNumber? = null
        var setkI: LoggedNetworkNumber? = null
        var setkD: LoggedNetworkNumber? = null

        init {
            if (kP < 0 || kI < 0 || kD < 0) {
                throw IllegalArgumentException("PID values must be positive")
            }
        }

        override fun makeTunable(key: String): PID {
            super.makeTunable(key)
            setkP = LoggedNetworkNumber("$key/kp", kP)
            setkI = LoggedNetworkNumber("$key/ki", kI)
            setkD = LoggedNetworkNumber("$key/kd", kD)
            return this
        }

        override fun update() {
            if (apply != null && !DriverStation.isFMSAttached()) {
                if (apply!!.get()) {
                    kP = setkP!!.get()
                    kI = setkI!!.get()
                    kD = setkD!!.get()
                    apply!!.set(false)
                    for (a in applyCallback) {
                        a.run()
                    }
                }
            }
        }
    }

    /**
     * simple class for PID gains, includes max speed and max acceleration
     */
    class ProfiledPID(
        @JvmField var kP: Double,
        @JvmField var kI: Double,
        @JvmField var kD: Double,
        @JvmField var maxSpeed: Double,
        @JvmField var maxAccel: Double
    ) : Gains() {
        var setkP: LoggedNetworkNumber? = null
        var setkI: LoggedNetworkNumber? = null
        var setkD: LoggedNetworkNumber? = null
        var setmaxSpeed: LoggedNetworkNumber? = null
        var setmaxAccel: LoggedNetworkNumber? = null

        init {
            if (kP < 0 || kI < 0 || kD < 0) {
                throw IllegalArgumentException("PID values must be positive")
            }
        }

        fun getPID(): PID {
            return PID(kP, kI, kD)
        }

        override fun makeTunable(key: String): ProfiledPID {
            super.makeTunable(key)
            setkP = LoggedNetworkNumber("$key/kp", kP)
            setkI = LoggedNetworkNumber("$key/ki", kI)
            setkD = LoggedNetworkNumber("$key/kd", kD)
            setmaxSpeed = LoggedNetworkNumber("$key/max speed", maxSpeed)
            setmaxAccel = LoggedNetworkNumber("$key/max acceleration", maxAccel)
            return this
        }

        override fun update() {
            if (apply != null && !DriverStation.isFMSAttached()) {
                if (apply!!.get()) {
                    kP = setkP!!.get()
                    kI = setkI!!.get()
                    kD = setkD!!.get()
                    maxSpeed = setmaxSpeed!!.get()
                    maxAccel = setmaxAccel!!.get()
                    apply!!.set(false)
                    for (a in applyCallback) {
                        a.run()
                    }
                }
            }
        }
    }

    /**
     * simple class for feedforward gains
     */
    class SimpleFF(
        @JvmField var kS: Double,
        @JvmField var kV: Double,
        @JvmField var kA: Double
    ) : Gains() {
        var setkS: LoggedNetworkNumber? = null
        var setkV: LoggedNetworkNumber? = null
        var setkA: LoggedNetworkNumber? = null

        init {
            if (kS < 0 || kV < 0 || kA < 0) {
                throw IllegalArgumentException("FF values must be positive")
            }
        }

        override fun makeTunable(key: String): SimpleFF {
            super.makeTunable(key)
            setkS = LoggedNetworkNumber("$key/ks", kS)
            setkV = LoggedNetworkNumber("$key/kv", kV)
            setkA = LoggedNetworkNumber("$key/ka", kA)
            return this
        }

        override fun update() {
            if (apply != null && !DriverStation.isFMSAttached()) {
                if (apply!!.get()) {
                    kS = setkS!!.get()
                    kV = setkV!!.get()
                    kA = setkA!!.get()
                    apply!!.set(false)
                    for (a in applyCallback) {
                        a.run()
                    }
                }
            }
        }

        fun copy(): SimpleFF {
            return SimpleFF(kS, kV, kA)
        }

        fun copyWithKs(ks: Double): SimpleFF {
            return SimpleFF(ks, kV, kA)
        }
    }

    /**
     * simple class for feedforward gains, includes kG for elevator and arm ff
     */
    class GravityFF : Gains {
        @JvmField var kS: Double
        @JvmField var kV: Double
        @JvmField var kA: Double
        @JvmField var kG: Double
        var setkS: LoggedNetworkNumber? = null
        var setkV: LoggedNetworkNumber? = null
        var setkA: LoggedNetworkNumber? = null
        var setkG: LoggedNetworkNumber? = null

        constructor(kS: Double, kG: Double, kV: Double, kA: Double) {
            this.kS = kS
            this.kV = kV
            this.kA = kA
            this.kG = kG
            if (kS < 0 || kV < 0 || kA < 0) {
                throw IllegalArgumentException("FF values must be positive")
            }
        }

        override fun makeTunable(key: String): GravityFF {
            super.makeTunable(key)
            setkS = LoggedNetworkNumber("$key/ks", kS)
            setkV = LoggedNetworkNumber("$key/kv", kV)
            setkA = LoggedNetworkNumber("$key/ka", kA)
            setkG = LoggedNetworkNumber("$key/kg", kG)
            return this
        }

        override fun update() {
            if (apply != null && !DriverStation.isFMSAttached()) {
                if (apply!!.get()) {
                    kS = setkS!!.get()
                    kV = setkV!!.get()
                    kA = setkA!!.get()
                    kG = setkG!!.get()
                    apply!!.set(false)
                    for (a in applyCallback) {
                        a.run()
                    }
                }
            }
        }
    }
}
